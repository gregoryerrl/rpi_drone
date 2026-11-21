#!/usr/bin/env python3
# cam_server.py — MJPEG stream of the Pi camera (IMX708) over HTTP
#   /usr/bin/python3 cam_server.py [port]      then open http://<pi-ip>:8000/
import io
import sys
from http import server
from threading import Condition, Lock

PAGE = b"""<html><head><title>Drone cam</title></head>
<body style="margin:0;background:#000;display:flex;justify-content:center">
<img src="stream.mjpg" style="max-width:100%;max-height:100vh"></body></html>"""


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.sequence = 0
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.sequence += 1
            self.condition.notify_all()
        return len(buf)


class CameraPreview:
    def __init__(self):
        self.lock = Lock()
        self.camera = None
        self.output = None
        self.viewers = 0
        self.closed = False

    def subscribe(self):
        with self.lock:
            if self.closed:
                raise RuntimeError("Server shutting down")
            if self.camera is None:
                from libcamera import ColorSpace
                from picamera2 import Picamera2
                from picamera2.encoders import MJPEGEncoder
                from picamera2.outputs import FileOutput

                camera = Picamera2()
                try:
                    output = StreamingOutput()
                    # The ISP supplies the full-range YUV420 that the MJPEG encoder consumes.
                    camera.configure(camera.create_video_configuration(
                        main={"size": (1280, 720), "format": "YUV420"}, colour_space=ColorSpace.Sycc()))
                    camera.start_recording(MJPEGEncoder(), FileOutput(output))
                except Exception:
                    camera.close()
                    raise
                self.camera, self.output = camera, output
            self.viewers += 1
            return self.output

    def unsubscribe(self):
        with self.lock:
            self.viewers -= 1
            if self.viewers == 0:
                self._stop()

    def close(self):
        with self.lock:
            self.closed = True
            self._stop()

    def _stop(self):
        camera, self.camera = self.camera, None
        self.output = None
        if camera is not None:
            try:
                camera.stop_recording()
            finally:
                camera.close()


class Handler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", len(PAGE))
            self.end_headers()
            self.wfile.write(PAGE)
        elif self.path == "/stream.mjpg":
            self.stream()
        else:
            self.send_error(404)

    def stream(self):
        try:
            output = self.server.preview.subscribe()
        except (OSError, RuntimeError):
            return self.send_error(503, "Camera unavailable")
        try:
            self.connection.settimeout(5)
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            seen = 0
            while True:
                with output.condition:
                    if not output.condition.wait_for(lambda: output.sequence != seen, timeout=5):
                        break
                    seen, frame = output.sequence, output.frame
                self.wfile.write(b"--FRAME\r\nContent-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
        except (ConnectionError, TimeoutError):
            pass
        finally:
            self.server.preview.unsubscribe()

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    with server.ThreadingHTTPServer(("", port), Handler) as httpd:
        httpd.preview = CameraPreview()
        print(f"Streaming on http://0.0.0.0:{port}/", flush=True)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            httpd.preview.close()
