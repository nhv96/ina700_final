"""Lightweight REST bridge between the detection program and the ABB robot.

The Raspberry Pi keeps a TCP server open for the robot (RAPID is the client) and
exposes a small HTTP API for the detection program:

    POST /position   {"x": 12.5, "y": 40.2, "shape": "star", "angle": 35.0}
                     -> sends "12.5,40.2,star,35.0" to the robot
    POST /disconnect -> sends "stop" to the robot, then closes the connection gracefully
    GET  /health     -> {"robot_connected": true|false}

For safety reasons and to avoid connection issues, this script must be run before loading the RAPID
program (starting the robot). The robot may reconnect at any time; requests made while it is not
connected are rejected with HTTP 503. On shutdown (Ctrl+C or SIGTERM) "stop" is sent as well.
"""
import argparse
import json
import math
import signal
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROBOT_HOST = "192.168.125.203"
ROBOT_PORT = 5000
API_HOST = "0.0.0.0"
API_PORT = 8000

# Must match the shape names understood by pickandplace2 in rapid/lab2.mod
SHAPES = {"hexagon", "star", "square", "triangle"}
MAX_BODY_BYTES = 1024


class RobotLink:
    """Owns the TCP connection to the robot. Thread safe."""

    def __init__(self, host, port):
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Re-use the port so restarting the script doesn't throw "Address already in use" errors
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((host, port))
        self._server.listen(1)
        self._client = None
        self._lock = threading.Lock()

    @property
    def connected(self):
        with self._lock:
            return self._client is not None

    def accept_forever(self):
        """Accept robot connections; a new connection replaces the previous one."""
        print("Waiting for ABB Robot to connect...")
        while True:
            try:
                client, address = self._server.accept()
            except OSError:
                return  # server socket closed
            print(f"Robot connected from IP: {address}")
            with self._lock:
                self._close_client()
                self._client = client
            threading.Thread(target=self._read_forever, args=(client,), daemon=True).start()
            self.send("connection_established now")

    def _read_forever(self, client):
        """Print every message the robot sends back (e.g. "DONE"); detect when it disconnects."""
        while True:
            try:
                data = client.recv(1024)
            except OSError:
                break
            if not data:
                break
            print(f"Received: {data.decode('utf-8', errors='replace')}")
        with self._lock:
            # Only clean up if this is still the active connection (not replaced or already closed)
            if self._client is client:
                print("Robot closed the connection.")
                self._close_client()

    def send(self, message):
        """Send a message to the robot. Returns False if the robot is not reachable."""
        with self._lock:
            if self._client is None:
                return False
            try:
                self._client.sendall(message.encode("utf-8"))
            except OSError as exc:
                print(f"Lost connection to robot: {exc}")
                self._close_client()
                return False
        print(f"Sent: {message}")
        return True

    def disconnect(self):
        """Send "stop" to the robot, then close the connection. Returns False if not connected."""
        with self._lock:
            if self._client is None:
                return False
            try:
                self._client.sendall(b"stop")
                print("Sent: stop")
                self._client.shutdown(socket.SHUT_WR)
            except OSError as exc:
                print(f"Could not send stop to robot: {exc}")
            self._close_client()
        print("Robot disconnected.")
        return True

    def close(self):
        self.disconnect()
        self._server.close()

    def _close_client(self):
        if self._client is not None:
            try:
                self._client.close()
            except OSError:
                pass
            self._client = None


def parse_position(payload):
    """Validate the JSON payload and return the "x,y,shape,angle" string for the robot."""
    if not isinstance(payload, dict):
        raise ValueError("body must be a JSON object")
    missing = [key for key in ("x", "y", "shape", "angle") if key not in payload]
    if missing:
        raise ValueError(f"missing field(s): {', '.join(missing)}")

    numbers = {}
    for key in ("x", "y", "angle"):
        value = payload[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"'{key}' must be a finite number")
        numbers[key] = round(float(value), 2)

    shape = payload["shape"]
    if not isinstance(shape, str) or shape.strip().lower() not in SHAPES:
        raise ValueError(f"'shape' must be one of: {', '.join(sorted(SHAPES))}")

    return f"{numbers['x']},{numbers['y']},{shape.strip().lower()},{numbers['angle']}"


def make_handler(robot):
    class Handler(BaseHTTPRequestHandler):
        def _reply(self, status, body):
            data = json.dumps(body).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _read_json(self):
            try:
                length = int(self.headers.get("Content-Length", 0))
            except ValueError:
                raise ValueError("invalid Content-Length")
            if length <= 0 or length > MAX_BODY_BYTES:
                raise ValueError(f"body must be 1-{MAX_BODY_BYTES} bytes")
            try:
                return json.loads(self.rfile.read(length))
            except json.JSONDecodeError:
                raise ValueError("body is not valid JSON")

        def _forward(self, message):
            if robot.send(message):
                self._reply(200, {"status": "sent", "message": message})
            else:
                self._reply(503, {"error": "robot not connected"})

        def do_GET(self):
            if self.path == "/health":
                self._reply(200, {"robot_connected": robot.connected})
            else:
                self._reply(404, {"error": "not found"})

        def do_POST(self):
            if self.path == "/position":
                try:
                    message = parse_position(self._read_json())
                except ValueError as exc:
                    self._reply(400, {"error": str(exc)})
                    return
                self._forward(message)
            elif self.path == "/disconnect":
                if robot.disconnect():
                    self._reply(200, {"status": "disconnected"})
                else:
                    self._reply(503, {"error": "robot not connected"})
            else:
                self._reply(404, {"error": "not found"})

        def log_message(self, format, *args):
            print(f"{self.address_string()} - {format % args}")

    return Handler


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="REST bridge to the ABB robot")
    parser.add_argument("--robot-host", default=ROBOT_HOST, help=f"IP to listen on for the robot (default: {ROBOT_HOST})")
    parser.add_argument("--robot-port", type=int, default=ROBOT_PORT, help=f"TCP port for the robot (default: {ROBOT_PORT})")
    parser.add_argument("--api-host", default=API_HOST, help=f"REST API bind address (default: {API_HOST})")
    parser.add_argument("--api-port", type=int, default=API_PORT, help=f"REST API port (default: {API_PORT})")
    args = parser.parse_args()

    robot = RobotLink(args.robot_host, args.robot_port)
    threading.Thread(target=robot.accept_forever, daemon=True).start()

    # docker stop / systemd send SIGTERM: shut down the same way as Ctrl+C
    def raise_interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, raise_interrupt)

    api = ThreadingHTTPServer((args.api_host, args.api_port), make_handler(robot))
    print(f"REST API listening on {args.api_host}:{args.api_port}")
    try:
        api.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        api.server_close()
        robot.close()
        print("Sockets closed.")
