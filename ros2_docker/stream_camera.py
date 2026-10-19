#!/usr/bin/env python3
"""
Mac Host: Webcam Streamer for ROS 2 (Docker).
Captures Mac webcam (FaceTime HD / Continuity Camera / USB webcam) via OpenCV
and streams an ultra-low latency MJPEG stream over HTTP (port 5006).

View in browser: http://localhost:5006
ROS 2 Stream:    http://host.docker.internal:5006/video
"""

from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import threading
import time
import sys
import os
import cv2


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    latest_jpeg = None
    running = True


class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/video':
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while self.server.running:
                    if self.server.latest_jpeg is not None:
                        frame = self.server.latest_jpeg
                        self.wfile.write(b'--FRAME\r\n')
                        self.send_header('Content-Type', 'image/jpeg')
                        self.send_header('Content-Length', str(len(frame)))
                        self.end_headers()
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                    time.sleep(0.033)  # ~30 FPS
            except Exception:
                pass
        elif self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            html = """<!DOCTYPE html>
<html>
<head>
    <title>Mac Webcam ➔ ROS 2 Streamer</title>
    <style>
        body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; background: #0f172a; color: #f8fafc; text-align: center; margin: 0; padding: 20px; }
        .card { background: #1e293b; max-width: 720px; margin: 0 auto; padding: 24px; border-radius: 12px; box-shadow: 0 10px 25px rgba(0,0,0,0.5); border: 1px solid #334155; }
        h1 { margin-top: 0; color: #38bdf8; font-size: 24px; }
        img { width: 100%; max-width: 640px; border-radius: 8px; border: 2px solid #38bdf8; }
        .tag { display: inline-block; background: #0284c7; padding: 4px 12px; border-radius: 20px; font-size: 13px; font-weight: bold; margin-bottom: 16px; }
        .info { margin-top: 16px; font-size: 14px; color: #94a3b8; }
    </style>
</head>
<body>
    <div class="card">
        <h1>📹 Mac Webcam Live Stream</h1>
        <div class="tag">Streaming to ROS 2 (/camera/image_raw)</div>
        <div>
            <img src="/video" alt="Webcam Stream" />
        </div>
        <div class="info">
            Docker URL: <code>http://host.docker.internal:5006/video</code><br>
            Topic in ROS 2: <code>/camera/image_raw</code>
        </div>
    </div>
</body>
</html>"""
            self.wfile.write(html.encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # Suppress standard HTTP request log spam


def capture_loop(server, camera_id=0):
    print(f"[*] Opening Mac camera (ID: {camera_id})...")
    cap = cv2.VideoCapture(camera_id)

    # Low latency 640x480 resolution at 30 fps
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print(f"[-] Failed to open camera ID {camera_id}.")
        print("    If macOS prompts for Camera permission, please click 'Allow'.")
        server.running = False
        return

    print(f"[+] Camera {camera_id} opened! Capturing 640x480 @ 30 FPS.")
    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 80]

    while server.running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue
        ret, jpeg = cv2.imencode('.jpg', frame, encode_params)
        if ret:
            server.latest_jpeg = jpeg.tobytes()
        time.sleep(0.015)

    cap.release()


def main():
    camera_id = 0
    if len(sys.argv) > 1:
        try:
            camera_id = int(sys.argv[1])
        except ValueError:
            pass

    port = 5006
    server = ThreadedHTTPServer(('0.0.0.0', port), StreamingHandler)
    server.running = True

    cap_thread = threading.Thread(target=capture_loop, args=(server, camera_id), daemon=True)
    cap_thread.start()

    print("==========================================================")
    print("      Mac Webcam ➔ ROS 2 (Docker) Streamer")
    print("==========================================================")
    print(f"  Local Web Preview: http://localhost:{port}")
    print(f"  Docker ROS 2 URL:  http://host.docker.internal:{port}/video")
    print("  Press Ctrl+C to stop.")
    print("==========================================================\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping camera stream...")
    finally:
        server.running = False
        server.server_close()


if __name__ == '__main__':
    main()
