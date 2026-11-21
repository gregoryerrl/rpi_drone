#!/usr/bin/env python3
# cam_server.py — MJPEG stream of the Pi camera (IMX708) over HTTP
#   /usr/bin/python3 cam_server.py [port]      then open http://<pi-ip>:8000/
# Uses system picamera2 (not in the venv).
import io, socketserver, sys
from http import server
from threading import Condition
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
PAGE = b"""<html><head><title>Drone cam</title></head>
<body style="margin:0;background:#000;display:flex;justify-content:center">
<img src="stream.mjpg" style="max-width:100%;max-height:100vh"></body></html>"""


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame, self.condition = None, Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class Handler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", len(PAGE))
            self.end_headers()
            self.wfile.write(PAGE)
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b"--FRAME\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)

    def log_message(self, *a):
        pass


class Server(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (1280, 720)}))
output = StreamingOutput()
picam2.start_recording(MJPEGEncoder(), FileOutput(output))
try:
    print(f"Streaming on http://0.0.0.0:{PORT}/", flush=True)
    Server(("", PORT), Handler).serve_forever()
finally:
    picam2.stop_recording()
