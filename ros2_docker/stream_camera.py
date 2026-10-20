#!/usr/bin/env python3
"""
Mac Host: Camera Streamer for ROS 2 (Docker).
Streams Mac webcam (or animated test pattern fallback if camera permissions are pending)
over both:
  1. Ultra-low latency raw TCP stream on port 5007 (for ROS 2 camera_publisher in Docker)
  2. Web browser MJPEG preview on port 5006 (http://localhost:5006)

Works out-of-the-box even before camera permissions are granted by providing a
live diagnostic test card so ROS 2 and RViz 2 never hang or crash.
"""

from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import math
import os
import socket
import struct
import sys
import threading
import time
import cv2
import numpy as np


class SharedFrameBuffer:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.running = True
        self.camera_active = False
        self.fps = 0.0


frame_buffer = SharedFrameBuffer()


def generate_test_pattern(frame_idx):
    """Generate an informative, animated test card if webcam is unavailable."""
    width, height = 640, 480
    img = np.zeros((height, width, 3), dtype=np.uint8)

    # Dark slate background with subtle scanlines
    for y in range(0, height, 4):
        img[y:y+2, :, :] = (15, 23, 42)
    img[img == 0] = 30

    # Animated radar / reticle
    cx, cy = 320, 200
    radius = 70
    cv2.circle(img, (cx, cy), radius, (56, 189, 248), 2)
    cv2.circle(img, (cx, cy), radius // 2, (56, 189, 248), 1)
    cv2.line(img, (cx - radius - 10, cy), (cx + radius + 10, cy), (56, 189, 248), 1)
    cv2.line(img, (cx, cy - radius - 10), (cx, cy + radius + 10), (56, 189, 248), 1)

    angle = (frame_idx * 5) % 360
    rad = math.radians(angle)
    px = int(cx + radius * math.cos(rad))
    py = int(cy + radius * math.sin(rad))
    cv2.line(img, (cx, cy), (px, py), (14, 165, 233), 2)

    # Text overlays
    cv2.putText(img, "myCobot 280 Camera Feed", (150, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)

    status_str = "STANDBY: Camera Permission Pending"
    cv2.putText(img, status_str, (120, 310),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (251, 191, 36), 2, cv2.LINE_AA)

    hint_str = "Enable in: System Settings -> Privacy & Security -> Camera"
    cv2.putText(img, hint_str, (50, 345),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (148, 163, 184), 1, cv2.LINE_AA)

    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    cv2.putText(img, f"Time: {ts}  |  Frame: {frame_idx}", (150, 440),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 116, 139), 1, cv2.LINE_AA)

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
    _, jpeg = cv2.imencode('.jpg', img, encode_params)
    return jpeg.tobytes()


class CameraManager:
    def __init__(self, camera_id=0):
        self.camera_id = camera_id
        self.cap = None
        self.lock = threading.Lock()
        self.should_connect = True
        self.running = True

    def start_background_connect(self):
        t = threading.Thread(target=self._connect_worker, daemon=True)
        t.start()

    def _connect_worker(self):
        while self.running and frame_buffer.running:
            if self.should_connect and (self.cap is None or not self.cap.isOpened()):
                try:
                    test_cap = cv2.VideoCapture(self.camera_id)
                    if test_cap.isOpened():
                        ret, test_frame = test_cap.read()
                        if ret and test_frame is not None:
                            test_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                            test_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                            test_cap.set(cv2.CAP_PROP_FPS, 30)
                            with self.lock:
                                self.cap = test_cap
                                self.should_connect = False
                            frame_buffer.camera_active = True
                            print(f"[+] Successfully opened physical camera {self.camera_id}!")
                        else:
                            test_cap.release()
                    else:
                        test_cap.release()
                except Exception:
                    pass
            time.sleep(4.0)

    def read_frame(self):
        with self.lock:
            if self.cap is not None and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    return frame
                else:
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                    self.cap = None
                    self.should_connect = True
                    frame_buffer.camera_active = False
        return None

    def release(self):
        self.running = False
        with self.lock:
            if self.cap is not None:
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None


def capture_loop(camera_id=0):
    """Main capture loop: uses physical camera if accessible, or falls back to test card."""
    print(f"[*] Initializing camera capture (ID: {camera_id})...")
    cam_mgr = CameraManager(camera_id)
    cam_mgr.start_background_connect()

    frame_count = 0
    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
    t_prev = time.time()

    while frame_buffer.running:
        frame = cam_mgr.read_frame()
        jpeg_bytes = None
        if frame is not None:
            # Enforce max 640x480 resolution (Mac webcams often force 1080p, destroying CPU)
            h, w = frame.shape[:2]
            if w > 640:
                scale = 640.0 / w
                frame = cv2.resize(frame, (640, int(h * scale)), interpolation=cv2.INTER_AREA)
            ret_enc, jpeg = cv2.imencode('.jpg', frame, encode_params)
            if ret_enc:
                jpeg_bytes = jpeg.tobytes()

        # Fallback to animated test pattern if camera hardware is busy or permission pending
        if jpeg_bytes is None:
            jpeg_bytes = generate_test_pattern(frame_count)

        with frame_buffer.lock:
            frame_buffer.latest_jpeg = jpeg_bytes

        frame_count += 1
        dt = time.time() - t_prev
        t_prev = time.time()
        if dt > 0:
            frame_buffer.fps = 0.9 * frame_buffer.fps + 0.1 * (1.0 / dt)

        time.sleep(0.066)  # 15 FPS: optimal for robotics & vision, eliminates high CPU load

    cam_mgr.release()


class TCPStreamServer:
    """Ultra-low latency TCP frame broadcaster for ROS 2 camera_publisher node."""
    def __init__(self, port=5007):
        self.port = port
        self.clients = []
        self.clients_lock = threading.Lock()
        self.running = True

    def start(self):
        t = threading.Thread(target=self._server_loop, daemon=True)
        t.start()
        b = threading.Thread(target=self._broadcast_loop, daemon=True)
        b.start()

    def _server_loop(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.port))
        server.listen(5)
        server.settimeout(1.0)
        print(f"[+] TCP Frame Server listening on 0.0.0.0:{self.port} (for ROS 2 Docker)")

        while self.running:
            try:
                conn, addr = server.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.settimeout(2.0)
                with self.clients_lock:
                    self.clients.append(conn)
                print(f"[+] ROS 2 client connected to camera TCP stream from {addr[0]}")
            except socket.timeout:
                continue
            except Exception:
                break
        server.close()

    def _broadcast_loop(self):
        last_frame_bytes = None
        while self.running:
            with frame_buffer.lock:
                frame_bytes = frame_buffer.latest_jpeg

            if frame_bytes is not None and frame_bytes is not last_frame_bytes:
                last_frame_bytes = frame_bytes
                # 4-byte big-endian length prefix followed by JPEG payload
                header = struct.pack('>I', len(frame_bytes))
                packet = header + frame_bytes

                with self.clients_lock:
                    dead_clients = []
                    for c in self.clients:
                        try:
                            c.sendall(packet)
                        except Exception:
                            dead_clients.append(c)
                    for d in dead_clients:
                        try:
                            d.close()
                        except Exception:
                            pass
                        self.clients.remove(d)

            time.sleep(0.015)


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class WebPreviewHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/video':
            self.send_response(200)
            self.send_header('Age', '0')
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while frame_buffer.running:
                    with frame_buffer.lock:
                        frame = frame_buffer.latest_jpeg
                    if frame is not None:
                        chunk = (
                            b'--FRAME\r\n'
                            b'Content-Type: image/jpeg\r\n'
                            b'Content-Length: ' + str(len(frame)).encode('ascii') + b'\r\n\r\n' +
                            frame +
                            b'\r\n'
                        )
                        self.wfile.write(chunk)
                    time.sleep(0.033)
            except Exception:
                pass
        elif self.path in ('/', '/index.html'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            html = """<!DOCTYPE html>
<html>
<head>
    <title>myCobot 280 Camera Stream</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; background: #0f172a; color: #f8fafc; text-align: center; margin: 0; padding: 24px; }
        .card { background: #1e293b; max-width: 720px; margin: 0 auto; padding: 24px; border-radius: 12px; box-shadow: 0 10px 25px rgba(0,0,0,0.5); border: 1px solid #334155; }
        h1 { margin-top: 0; color: #38bdf8; font-size: 24px; }
        img { width: 100%; max-width: 640px; border-radius: 8px; border: 2px solid #38bdf8; background: #000; }
        .tag { display: inline-block; background: #0284c7; padding: 4px 14px; border-radius: 20px; font-size: 13px; font-weight: bold; margin-bottom: 16px; }
        .info { margin-top: 18px; font-size: 14px; color: #94a3b8; text-align: left; background: #0f172a; padding: 12px 16px; border-radius: 8px; }
        code { color: #38bdf8; font-family: monospace; }
    </style>
</head>
<body>
    <div class="card">
        <h1>📹 Mac Camera ➔ ROS 2 Streamer</h1>
        <div class="tag">Streaming to /camera/image_raw</div>
        <div>
            <img src="/video" alt="Webcam Stream" />
        </div>
        <div class="info">
            • <b>ROS 2 TCP Bridge:</b> <code>host.docker.internal:5007</code> (Ultra-low latency)<br>
            • <b>ROS 2 Raw Topic:</b> <code>/camera/image_raw</code><br>
            • <b>ROS 2 Compressed:</b> <code>/camera/image_raw/compressed</code>
        </div>
    </div>
</body>
</html>"""
            self.wfile.write(html.encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass


def main():
    camera_id = 0
    if len(sys.argv) > 1:
        try:
            camera_id = int(sys.argv[1])
        except ValueError:
            pass

    http_port = 5006
    tcp_port = 5007

    # 1. Start Capture Thread
    cap_thread = threading.Thread(target=capture_loop, args=(camera_id,), daemon=True)
    cap_thread.start()

    # 2. Start TCP Server (for Docker ROS 2 node)
    tcp_server = TCPStreamServer(port=tcp_port)
    tcp_server.start()

    # 3. Start HTTP Server (for Browser Preview)
    http_server = ThreadedHTTPServer(('0.0.0.0', http_port), WebPreviewHandler)

    print("==========================================================")
    print("      Mac Camera ➔ ROS 2 (Docker) Streamer")
    print("==========================================================")
    print(f"  Web Browser Preview: http://localhost:{http_port}")
    print(f"  ROS 2 TCP Stream:    0.0.0.0:{tcp_port}")
    print("  Status: Active and streaming to ROS 2 container.")
    print("  Press Ctrl+C to stop.")
    print("==========================================================\n")

    try:
        http_server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping camera streamer...")
    finally:
        frame_buffer.running = False
        tcp_server.running = False
        http_server.server_close()


if __name__ == '__main__':
    main()
