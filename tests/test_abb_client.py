import socket
import threading
import unittest

from python.abb_client import (
    AbbTcpClient,
    BridgeConfig,
    ProtocolError,
    ResponseBuffer,
    encode_request,
    parse_joints,
)


class ProtocolTests(unittest.TestCase):
    def test_encode_request_is_exactly_64_bytes(self):
        frame = encode_request("  move   home  ")
        self.assertEqual(len(frame), 64)
        self.assertTrue(frame.startswith(b"MOVE HOME "))
        self.assertEqual(frame.rstrip(b" "), b"MOVE HOME")

    def test_encode_request_rejects_non_ascii(self):
        with self.assertRaises(ValueError):
            encode_request("移动")

    def test_response_buffer_handles_fragmented_and_combined_frames(self):
        responses = ResponseBuffer()
        responses.feed(b"OK ACC")
        self.assertIsNone(responses.pop())
        responses.feed(b"EPTED HOME#OK DONE HOME#")
        self.assertEqual(responses.pop(), "OK ACCEPTED HOME#")
        self.assertEqual(responses.pop(), "OK DONE HOME#")

    def test_response_buffer_rejects_oversized_frame(self):
        responses = ResponseBuffer()
        with self.assertRaises(ProtocolError):
            responses.feed(b"X" * 81)

    def test_response_buffer_checks_remainder_after_complete_frame(self):
        responses = ResponseBuffer()
        responses.feed(b"OK PONG#" + b"X" * 81)
        self.assertEqual(responses.pop(), "OK PONG#")
        with self.assertRaises(ProtocolError):
            responses.pop()

    def test_parse_six_joint_values(self):
        self.assertEqual(
            parse_joints("OK JOINTS 0.00,-2.00,3.50,4.00,5.00,6.00#"),
            [0.0, -2.0, 3.5, 4.0, 5.0, 6.0],
        )

    def test_parse_joints_rejects_wrong_count(self):
        with self.assertRaises(ProtocolError):
            parse_joints("OK JOINTS 1,2,3#")


class FakeBridge:
    def __init__(self, responses):
        self.responses = responses
        self.requests = []
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.host, self.port = self.listener.getsockname()
        self.thread = threading.Thread(target=self._serve, daemon=True)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.listener.close()
        self.thread.join(timeout=1)

    def _serve(self):
        connection, _ = self.listener.accept()
        with connection:
            for response_chunks in self.responses:
                request = bytearray()
                while len(request) < 64:
                    chunk = connection.recv(64 - len(request))
                    if not chunk:
                        return
                    request.extend(chunk)
                self.requests.append(bytes(request))
                for chunk in response_chunks:
                    connection.sendall(chunk)


class ClientIntegrationTests(unittest.TestCase):
    def test_ping_and_fragmented_motion_responses(self):
        responses = [
            [b"OK PONG#"],
            [b"OK ACCEPTED ", b"HOME#OK DONE HOME#"],
        ]
        with FakeBridge(responses) as bridge:
            config = BridgeConfig(
                host=bridge.host,
                port=bridge.port,
                connect_timeout=1,
                response_timeout=1,
                motion_timeout=1,
            )
            with AbbTcpClient(config) as client:
                client.ping()
                client.move("HOME")

        self.assertEqual(bridge.requests[0].rstrip(), b"PING")
        self.assertEqual(bridge.requests[1].rstrip(), b"MOVE HOME")


if __name__ == "__main__":
    unittest.main()
