#!/usr/bin/env python3
"""
Host-side Serial <-> TCP Bridge for myCobot on macOS.
Bridges the Mac USB serial port to a TCP socket so Docker can access it as /dev/ttyUSB0.
"""

import glob
import os
import select
import socket
import sys
import time
import serial


def find_serial_port():
    patterns = [
        "/dev/cu.usbserial*",
        "/dev/cu.wchusbserial*",
        "/dev/cu.SLAB_USBtoUART*",
    ]
    matches = []
    for p in patterns:
        matches.extend(glob.glob(p))
    return matches


def main():
    baud = 115200
    host = "0.0.0.0"
    port = 5005

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    server.bind((host, port))
    server.listen(1)
    server.setblocking(False)
    print(f"[+] TCP server listening on {host}:{port} for Docker connection...")
    print("    In Docker, socat will connect to host.docker.internal:5005 -> /dev/ttyUSB0")

    client = None
    ser = None
    last_wait_log = 0.0

    while True:
        try:
            # 1. Accept Docker connection if not connected
            if client is None:
                try:
                    c, addr = server.accept()
                    c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    c.setblocking(False)
                    client = c
                    print(f"[+] Docker container connected from {addr}")
                except (BlockingIOError, socket.error):
                    pass

            # 2. Maintain Serial connection to myCobot
            if ser is None:
                ports = find_serial_port()
                if ports:
                    target_port = ports[0]
                    try:
                        ser = serial.Serial(target_port, baud, timeout=0.01, write_timeout=0.05)
                        print(f"[+] Serial port {target_port} opened at {baud} baud.")
                    except Exception as e:
                        ser = None
                        time.sleep(0.5)
                else:
                    now = time.time()
                    if now - last_wait_log > 5.0:
                        last_wait_log = now
                        print("[*] Waiting for myCobot USB-C cable to be connected to Mac and powered on...")
                    time.sleep(0.5)

            # 3. If both connected, bridge bidirectional data
            if client is not None and ser is not None:
                # Serial -> Docker Socket
                try:
                    if ser.in_waiting > 0:
                        data = ser.read(ser.in_waiting)
                        if data:
                            client.sendall(data)
                except Exception as e:
                    print(f"[-] Serial read disconnected: {e}")
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None

                # Docker Socket -> Serial
                r, _, _ = select.select([client], [], [], 0.001)
                if r:
                    try:
                        recv_data = client.recv(4096)
                        if not recv_data:
                            print("[-] Docker client disconnected.")
                            client.close()
                            client = None
                        else:
                            ser.write(recv_data)
                            ser.flush()
                    except Exception as e:
                        print(f"[-] Socket/Serial error: {e}")
                        if client:
                            client.close()
                            client = None

                time.sleep(0.001)  # 1ms micro-sleep to prevent 100% CPU while giving sub-5ms latency
            elif client is not None and ser is None:
                # Docker is connected but serial is waiting; drain socket discard or wait
                r, _, _ = select.select([client], [], [], 0.02)
                if r:
                    try:
                        recv_data = client.recv(4096)
                        if not recv_data:
                            print("[-] Docker client disconnected.")
                            client.close()
                            client = None
                    except Exception:
                        if client:
                            client.close()
                            client = None
            else:
                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\nExiting bridge.")
            break
        except Exception as e:
            print(f"[-] Error: {e}")
            time.sleep(1)

    if ser:
        ser.close()
    if client:
        client.close()
    server.close()


if __name__ == "__main__":
    main()
