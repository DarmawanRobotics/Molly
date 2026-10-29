"""The UDP socket: a reader thread that wakes the executor, datagram and stream sending."""

from __future__ import annotations

from collections import deque
import contextlib
import select
import socket
import threading

from molly_bridge.core import protocol

INBOX = 256
BURST = 64
SEND_BUFFER = 1 << 20  # a camera frame goes out as one burst of datagrams (capped by wmem_max)


class UdpLink:
    """Receives on the reader thread, hands datagrams to the executor through a guard."""

    def __init__(self, node, port: int, handle):
        self.handle = handle
        self.inbox: deque = deque(maxlen=INBOX)
        self.frames = dict.fromkeys(
            (protocol.STREAM_CAMERA, protocol.STREAM_LIDAR, protocol.STREAM_ROUTE,
             protocol.STREAM_MAP, protocol.STREAM_SITE), 0,
        )  # fmt: skip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        with contextlib.suppress(OSError):
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, SEND_BUFFER)
        self.sock.bind(('0.0.0.0', port))
        self.sock.setblocking(False)
        self.wake = node.create_guard_condition(self.drain)
        self.running = True
        self.reader = threading.Thread(target=self.read, name='teleop-udp', daemon=True)
        self.reader.start()

    def close(self):
        """Stop the reader and close the socket, so the port is free again at once."""
        self.running = False
        self.reader.join(timeout=1.0)
        self.sock.close()

    def read(self):
        """Receive datagrams and wake the executor to handle them."""
        while self.running:
            try:
                ready, _, _ = select.select([self.sock], [], [], 0.5)
            except (OSError, ValueError):  # socket closed under us
                return
            if not ready:
                continue
            for _ in range(BURST):
                try:
                    self.inbox.append(self.sock.recvfrom(65535))
                except (BlockingIOError, InterruptedError):
                    break
                except OSError:
                    if not self.running:
                        return
                    break
            try:
                self.wake.trigger()
            except Exception:  # rclpy is shutting down
                return

    def drain(self):
        """Handle the received datagrams (bounded inbox: a flood cannot starve ROS)."""
        for _ in range(len(self.inbox)):
            data, addr = self.inbox.popleft()
            message = protocol.decode_json(data)
            if message is not None:
                self.handle(message, addr)

    def send(self, data: bytes, addr) -> bool:
        """Send one datagram; False when it was dropped (socket busy or no address)."""
        if addr is None:
            return False
        try:
            self.sock.sendto(data, addr)
        except OSError:
            return False
        return True

    def send_json(self, message: dict, addr):
        """Send a JSON message."""
        self.send(protocol.encode_json(message), addr)

    def send_stream(self, stream: int, payload: bytes, addr):
        """Send a chunked binary payload; stop at a dropped chunk (the frame is lost anyway)."""
        self.frames[stream] += 1
        for datagram in protocol.chunk(stream, self.frames[stream], payload):
            if not self.send(datagram, addr):
                return
