"""Haras Amor camera preview with momentary, SSH-local motor bench control."""
import argparse
import asyncio
import getpass
import html
from importlib import import_module
import ipaddress
import json
from pathlib import Path
import re
import secrets
import time

from aiohttp import WSMsgType, web

from .motors import BenchError, DEFAULT_DEVICE, MAX_PERCENT, MotorLink


HERE = Path(__file__).resolve().parent
REPLY_TIMEOUT = 0.32
POLL_PAUSE = 0.10


def local_page(request):
    peer = request.remote
    return (peer is not None and ipaddress.ip_address(peer).is_loopback
            and re.fullmatch(r"(?:localhost|127\.0\.0\.1|\[::1\])(?::[0-9]{1,5})?", request.host) is not None)


async def message(ws, timeout):
    try:
        packet = await asyncio.wait_for(ws.receive(), timeout)
    except TimeoutError:
        return None
    if packet.type in (WSMsgType.CLOSE, WSMsgType.CLOSED, WSMsgType.CLOSING, WSMsgType.ERROR):
        raise ConnectionError("Operator disconnected")
    if packet.type == WSMsgType.TEXT:
        try:
            data = json.loads(packet.data)
            if isinstance(data, dict):
                return data
        except ValueError:
            pass
    raise BenchError("Invalid operator message.")


async def held_slider(ws, link, hold):
    while True:
        link.check()
        # A single outstanding challenge prevents buffered browser updates from
        # renewing output after an interruption. No fresh reply means no next pulse.
        ticket = secrets.token_hex(12)
        deadline = time.monotonic() + REPLY_TIMEOUT
        await ws.send_json({"hold": hold, "poll": ticket})
        reply = await message(ws, max(0, deadline - time.monotonic()))
        if reply == {"release": hold}:
            break
        if (reply is None or time.monotonic() >= deadline or reply.get("poll") != ticket
                or reply.get("hold") != hold):
            raise BenchError("Hold expired. Release the slider and reconnect.")
        await link.set_percent(reply.get("percent"))
        # Listen for release during the interval between refreshes.
        early = await message(ws, POLL_PAUSE)
        if early == {"release": hold}:
            break
        if early is not None:
            raise BenchError("Unexpected operator message.")
    await link.set_percent(0)
    await ws.send_json({"released": hold})


async def operate(ws, link):
    await link.ready()
    await ws.send_json({"ready": True})
    last_hold = 0
    while True:
        link.check()
        data = await message(ws, 0.4)
        if data is None:
            continue
        hold = data.get("hold")
        if set(data) != {"hold"} or type(hold) is not int or hold <= last_hold:
            raise BenchError("Expected a new slider press.")
        last_hold = hold
        await held_slider(ws, link, hold)


def next_frame(output, sequence):
    with output.condition:
        if output.condition.wait_for(lambda: output.sequence != sequence, timeout=4):
            return output.sequence, output.frame
        return sequence, None


class Preview:
    def __init__(self, camera, open_link):
        self.camera = camera
        self.open_link = open_link
        self.operator = None
        self.page = (HERE / "index.html").read_text().replace("__MAX__", str(MAX_PERCENT))
        self.page = self.page.replace("__USER__", html.escape(getpass.getuser(), quote=True))
        self.script = (HERE / "controls.js").read_bytes()

    async def index(self, request):
        page = self.page.replace("__LOCAL__", "yes" if local_page(request) else "no")
        return web.Response(text=page, content_type="text/html")

    async def javascript(self, request):
        return web.Response(body=self.script, content_type="application/javascript")

    async def stream(self, request):
        try:
            output = await asyncio.to_thread(self.camera.subscribe)
        except (RuntimeError, OSError, IndexError) as exc:
            raise web.HTTPServiceUnavailable(text="Camera unavailable") from exc
        response = web.StreamResponse(headers={"Content-Type": "multipart/x-mixed-replace; boundary=FRAME"})
        try:
            await response.prepare(request)
            sequence = 0
            while True:
                sequence, frame = await asyncio.to_thread(next_frame, output, sequence)
                if frame is None:
                    break
                header = f"--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: {len(frame)}\r\n\r\n"
                async with asyncio.timeout(4):
                    await response.write(header.encode() + frame + b"\r\n")
        except (ConnectionError, TimeoutError):
            pass
        finally:
            await asyncio.to_thread(self.camera.unsubscribe)
        return response

    async def motors(self, request):
        if not local_page(request) or request.headers.get("Origin") != "http://" + request.host:
            raise web.HTTPForbidden(text="Open the operator preview through SSH.")
        ws = web.WebSocketResponse(max_msg_size=512, compress=False)
        await ws.prepare(request)
        if self.operator is not None:
            await ws.send_json({"error": "Another operator has the motor control."})
            await ws.close()
            return ws
        self.operator = ws
        link = None
        error = None
        try:
            link = self.open_link()
            await operate(ws, link)
        except ConnectionError:
            pass
        except (BenchError, OSError) as exc:
            error = str(exc)
        finally:
            try:
                if link is not None:
                    link.close()
            finally:
                self.operator = None
        if error and not ws.closed:
            await ws.send_json({"error": error})
        await ws.close()
        return ws

    async def shutdown(self, app):
        if self.operator is not None:
            await self.operator.close(code=1001, message=b"Preview stopped")
        await asyncio.to_thread(self.camera.close)


async def headers(request, response):
    response.headers.update({
        "Cache-Control": "no-store",
        "Referrer-Policy": "no-referrer",
        "X-Content-Type-Options": "nosniff",
        "X-Frame-Options": "DENY",
        "Content-Security-Policy": "default-src 'none'; img-src 'self'; script-src 'self'; "
        "style-src 'unsafe-inline'; connect-src 'self' ws://localhost:* ws://127.0.0.1:*; "
        "frame-ancestors 'none'; base-uri 'none'",
    })


def make_app(camera, open_link):
    preview = Preview(camera, open_link)
    app = web.Application()
    for route in ("/", "/index.html", "/live"):
        app.router.add_get(route, preview.index)
    app.router.add_get("/controls.js", preview.javascript)
    app.router.add_get("/stream.mjpg", preview.stream, allow_head=False)
    app.router.add_get("/motors", preview.motors, allow_head=False)
    app.on_response_prepare.append(headers)
    app.on_shutdown.append(preview.shutdown)
    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    args = parser.parse_args()
    # Reuse the same on-demand MJPEG encoder as the standalone camera preview.
    camera_project = import_module("research.cv-research.pan-tilt-face-tracking.cam_server")
    app = make_app(camera_project.CameraPreview(), lambda: MotorLink(args.device))
    web.run_app(app, host=args.bind, port=args.port, access_log=None)


if __name__ == "__main__":
    main()
