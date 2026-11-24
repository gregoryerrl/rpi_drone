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

from .leveling import TRIAL_TILT_DEG
from .motors import BenchError, DEFAULT_DEVICE, HEARTBEAT_AGE, MAX_PERCENT, MotorLink, MotorStopped
from .stop import StopLatch, stop_marker


HERE = Path(__file__).resolve().parent
REPLY_TIMEOUT = 0.32
POLL_PAUSE = 0.10


def local_page(request):
    peer = request.remote
    return (peer is not None and ipaddress.ip_address(peer).is_loopback
            and re.fullmatch(r"(?:localhost|127\.0\.0\.1|\[::1\])(?::[0-9]{1,5})?", request.host) is not None)


def require_operator_origin(request):
    if not local_page(request) or request.headers.get("Origin") != "http://" + request.host:
        raise web.HTTPForbidden(text="Open the operator preview through SSH.")


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
        if link.outputs_stopped:
            return
        # A single outstanding challenge prevents buffered browser updates from
        # renewing output after an interruption. No fresh reply means no next pulse.
        ticket = secrets.token_hex(12)
        deadline = time.monotonic() + REPLY_TIMEOUT
        await ws.send_json({"hold": hold, "poll": ticket})
        reply = await message(ws, max(0, deadline - time.monotonic()))
        if link.outputs_stopped:
            return
        if reply == {"release": hold}:
            break
        if (reply is None or time.monotonic() >= deadline or reply.get("poll") != ticket
                or reply.get("hold") != hold):
            raise BenchError("Hold expired. Release the bench button and reconnect.")
        await link.set_percent(reply.get("percent"))
        # Listen for release during the interval between refreshes.
        early = await message(ws, POLL_PAUSE)
        if link.outputs_stopped:
            return
        if early == {"release": hold}:
            break
        if early is not None:
            raise BenchError("Unexpected operator message.")
    await link.set_percent(0)
    await ws.send_json({"released": hold})


async def run_jig(ws, link):
    try:
        link.jig.begin()
        while not link.outputs_stopped:
            ticket = secrets.token_hex(12)
            deadline = time.monotonic() + REPLY_TIMEOUT
            await ws.send_json({"jig_poll": ticket, **link.jig.status()})
            reply = await message(ws, max(0, deadline - time.monotonic()))
            if link.outputs_stopped:
                return
            if reply == {"jig": "off"}:
                break
            if (reply is None or time.monotonic() >= deadline
                    or set(reply) != {"jig_poll", "roll", "pitch", "throttle", "arm"}
                    or reply["jig_poll"] != ticket):
                raise ValueError("Jig input expired. Start a new session.")
            try:
                link.jig.update(reply["roll"], reply["pitch"], reply["throttle"], reply["arm"], MAX_PERCENT)
            except ValueError as exc:
                if reply["arm"] is True and not link.jig.arm_pending:
                    link.jig.record_arm("blocked", f"The Pi blocked the arm request. {exc}")
                raise
            await asyncio.sleep(.06)
    finally:
        link.jig.end()


async def operate(ws, link):
    await link.ready()
    await ws.send_json({"stopped": True} if link.outputs_stopped else {"ready": True})
    last_hold = 0
    while True:
        link.check()
        data = await message(ws, 0.4)
        if data is None or link.outputs_stopped:
            continue
        if data == {"jig": "on"}:
            reason = "Auto stabilize off."
            try:
                await run_jig(ws, link)
            except ValueError as exc:
                reason = str(exc)
            if not link.outputs_stopped:
                await ws.send_json({"jig_off": reason, "fc_stop": link.jig.stop_message,
                                    "fc_pending": link.jig.stop_pending})
                if link.jig.stop_pending:
                    while link.jig.stop_pending and not link.outputs_stopped:
                        link.check()
                        await message(ws, .2)
                    if not link.outputs_stopped:
                        await ws.send_json({"ready": True, "jig_disarmed": True})
            continue
        if "jig_poll" in data or data == {"jig": "off"}:
            continue  # An old jig ticket cannot start a bench-slider hold.
        hold = data.get("hold")
        if set(data) != {"hold"} or type(hold) is not int or hold <= last_hold:
            raise BenchError("Expected a new bench-button hold.")
        last_hold = hold
        try:
            await held_slider(ws, link, hold)
        except MotorStopped:
            pass  # Discard this hold; the inhibited session can still provide telemetry.


def next_frame(output, sequence):
    with output.condition:
        if output.condition.wait_for(lambda: output.sequence != sequence, timeout=4):
            return output.sequence, output.frame
        return sequence, None


class Preview:
    def __init__(self, camera, open_link, marker):
        self.camera = camera
        self.open_link = open_link
        self.operator = None
        self.operator_link = None
        self.last_arm_report = None
        self.latch = StopLatch(marker)
        self.jig_stop_message = ""
        self.page = (HERE / "index.html").read_text().replace("__MAX__", str(MAX_PERCENT))
        self.page = self.page.replace("__USER__", html.escape(getpass.getuser(), quote=True))
        self.page = self.page.replace("__TRIAL_TILT__", str(TRIAL_TILT_DEG))
        self.script = (HERE / "controls.js").read_bytes()
        self.leveling_script = (HERE / "leveling.js").read_bytes()

    async def index(self, request):
        page = self.page.replace("__LOCAL__", "yes" if local_page(request) else "no")
        return web.Response(text=page, content_type="text/html")

    async def javascript(self, request):
        return web.Response(body=self.script, content_type="application/javascript")

    async def leveling_javascript(self, request):
        return web.Response(body=self.leveling_script, content_type="application/javascript")

    async def leveling_state(self, request):
        link = self.operator_link
        operator_view = local_page(request)
        if link is None:
            state = {"message": "Open the SSH operator preview to receive FC telemetry."}
        else:
            try:
                link.check()
            except BenchError as exc:
                state = {"message": str(exc)}
            else:
                state = link.leveling.snapshot(time.monotonic(), *link.jig.target)
                if operator_view and link.jig.selected:
                    state["jig"] = link.jig.status()
        last = (link.jig.last_arm_report if link else None) or self.last_arm_report
        if operator_view and last:
            state["last_arm_attempt"] = last
        return web.json_response(state)

    async def stop_status(self, request):
        state = {"stopped": self.latch.stopped, "storage_error": self.latch.storage_error}
        if self.latch.stopped and local_page(request):
            state["token"] = self.latch.token
        if self.latch.stopped:
            link = self.operator_link
            if self.jig_stop_message and link is not None and not link.jig.stop_pending and not link.jig.armed:
                if link.system is not None and time.monotonic() - link.last_heartbeat < HEARTBEAT_AGE and not link.fault:
                    self.jig_stop_message = ""
                    link.jig.stop_message = "PX4 reports disarmed."
            state["fc_stop"] = (link.jig.stop_message if link else "") or self.jig_stop_message
        return web.json_response(state)

    async def stop_motors(self, request):
        require_operator_origin(request)
        self.latch.engage()
        try:
            if self.operator_link is not None:
                self.operator_link.stop_outputs()
        finally:
            self.latch.persist()
        operator = self.operator
        if operator is not None and not operator.closed:
            try:
                await asyncio.wait_for(operator.send_json({"stopped": True}), 0.2)
            except (OSError, RuntimeError, TimeoutError):
                pass  # Output is already inhibited, including on the old socket.
        return await self.stop_status(request)

    async def reset_stop(self, request):
        require_operator_origin(request)
        if self.jig_stop_message or (self.operator_link is not None and self.operator_link.jig.stop_pending):
            raise web.HTTPConflict(text="PX4 disarm is unconfirmed. Use the motor-power disconnect.")
        if request.content_type != "application/json":
            raise web.HTTPUnsupportedMediaType()
        try:
            data = await request.json()
        except (ValueError, UnicodeError) as exc:
            raise web.HTTPBadRequest() from exc
        try:
            reset = isinstance(data, dict) and self.latch.reset(data.get("token"))
        except OSError as exc:
            raise web.HTTPServiceUnavailable(text="Could not clear the stored motor stop.") from exc
        if not reset:
            raise web.HTTPConflict(text="This reset does not match the current stop.")
        self.jig_stop_message = ""
        # Old holds and buffered replies must never resume after reset.
        operator = self.operator
        if operator is not None:
            try:
                await asyncio.wait_for(operator.close(code=1001, message=b"Motor stop reset"), 0.6)
            except (OSError, TimeoutError):
                pass
        return await self.stop_status(request)

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
        require_operator_origin(request)
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
            self.operator_link = link
            if self.latch.stopped:
                link.stop_outputs()
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
                if link is not None and link.jig.last_arm_report:
                    self.last_arm_report = link.jig.last_arm_report
                if link is not None and link.jig.stop_pending:
                    self.jig_stop_message = link.jig.stop_message
                self.operator_link = None
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


def make_app(camera, open_link, marker=None):
    preview = Preview(camera, open_link, marker)
    app = web.Application()
    for route in ("/", "/index.html", "/live"):
        app.router.add_get(route, preview.index)
    app.router.add_get("/controls.js", preview.javascript)
    app.router.add_get("/leveling.js", preview.leveling_javascript)
    app.router.add_get("/api/leveling", preview.leveling_state)
    app.router.add_get("/api/motor-stop", preview.stop_status)
    app.router.add_post("/api/motor-stop", preview.stop_motors)
    app.router.add_post("/api/motor-stop/reset", preview.reset_stop)
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
    app = make_app(camera_project.CameraPreview(), lambda: MotorLink(args.device), stop_marker())
    web.run_app(app, host=args.bind, port=args.port, access_log=None)


if __name__ == "__main__":
    main()
