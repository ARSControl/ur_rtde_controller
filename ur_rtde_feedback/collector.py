"""Output-only RTDE ingestion with a bounded handoff to a separate publisher.

No robot input recipe, control interface, dashboard command or URScript is
used. This module does not infer robot-side packet loss or motion causation.
"""
import math
import queue
import re
import socket
import threading
import time
import uuid

from . import protocol


class TimestampRegression(protocol.ProtocolError):
    """Host or robot clock discontinuity makes this session unusable."""


class FeedbackCollector:
    """Own one connection session; disconnect/fault never auto-reconnects."""

    def __init__(self, host, frequency, capacity, expected_version='5.9.3', connector=None, timeout_s=3.0):
        if not isinstance(host, str) or not host or len(host) > 255:
            raise ValueError('robot host must be nonempty')
        if (isinstance(frequency, bool) or not isinstance(frequency, (int, float)) or
                not math.isfinite(frequency) or not 1 <= frequency <= 500):
            raise ValueError('frequency must be 1..500 Hz')
        if type(capacity) is not int or not 1 <= capacity <= 65536:
            raise ValueError('queue capacity must be 1..65536')
        if not isinstance(expected_version, str) or not re.fullmatch(r'\d+\.\d+\.\d+', expected_version):
            raise ValueError('expected RTDE controller version must be major.minor.patch')
        if (isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)) or
                not math.isfinite(timeout_s) or not 0.1 <= timeout_s <= 5.0):
            raise ValueError('socket timeout must be finite and between 0.1 and 5 seconds')
        self.host, self.frequency, self.expected_version = host, float(frequency), expected_version
        self.timeout_s = float(timeout_s)
        self.collector_session = str(uuid.uuid4())
        self.host_clock_id = protocol.host_clock_id() or ''
        self.queue = queue.Queue(maxsize=capacity)
        self._connect = connector or socket.create_connection
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._socket = None
        self._last_robot = None
        self._last_host = 0
        self._controller_version = ''
        self._recipe_id = 0
        self._stats = dict(connection_state='created', detail='', packets_received=0,
                           packets_published=0, local_queue_dropped=0, publication_failures=0,
                           timestamp_regressions=0, source_gap_observations=0,
                           malformed_packets=0, transport_errors=0, reconnects=0,
                           last_sequence=0)

    def _state(self, state, detail=''):
        with self._lock:
            self._stats['connection_state'], self._stats['detail'] = state, str(detail)[:512]

    def health(self):
        with self._lock:
            result = dict(self._stats)
        result.update(schema_version=1, collector_session=self.collector_session,
                      host_clock_id=self.host_clock_id, host_time_ns=time.monotonic_ns(),
                      pending_queue=self.queue.qsize(), source_loss_known=False)
        return result

    def _accept(self, sample, host_ns):
        if type(host_ns) is not int or host_ns <= 0:
            raise protocol.ProtocolError('invalid host packet read timestamp')
        timestamp = sample['timestamp']
        with self._lock:
            if timestamp < 0:
                self._stats['timestamp_regressions'] += 1
                raise TimestampRegression('negative robot timestamp; session invalid')
            if host_ns <= self._last_host:
                self._stats['timestamp_regressions'] += 1
                raise TimestampRegression('host read timestamp did not increase; session invalid')
            if self._last_robot is not None:
                delta = timestamp - self._last_robot
                if delta <= 0:
                    self._stats['timestamp_regressions'] += 1
                    raise TimestampRegression('robot timestamp did not increase; session invalid')
                if delta > 1.5 / self.frequency:
                    self._stats['source_gap_observations'] += 1
            self._last_robot = timestamp
            self._last_host = host_ns
            self._stats['packets_received'] += 1
            self._stats['last_sequence'] += 1
            sequence = self._stats['last_sequence']
        record = dict(schema_version=1, collector_session=self.collector_session,
                      sequence=sequence, robot_timestamp=timestamp,
                      host_receive_ns=host_ns, host_clock_id=self.host_clock_id,
                      requested_frequency_hz=self.frequency, controller_version=self._controller_version,
                      protocol_version=2, recipe_id=self._recipe_id,
                      recipe_fields=[name for name, _ in protocol.FIELDS],
                      actual_q=sample['actual_q'], actual_qd=sample['actual_qd'], target_qd=sample['target_qd'],
                      actual_tcp_pose=sample['actual_TCP_pose'], actual_tcp_speed=sample['actual_TCP_speed'],
                      speed_scaling=sample['speed_scaling'], robot_status_bits=sample['robot_status_bits'],
                      safety_status_bits=sample['safety_status_bits'], source_loss_known=False)
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            with self._lock:
                self._stats['local_queue_dropped'] += 1

    def receive_from(self, sock):
        """Negotiate/consume a socket-like object; suitable for fake-peer tests."""
        if self._stop.is_set():
            self._state('stopped')
            return
        self._state('connecting')
        try:
            version, recipe = protocol.setup(sock, self.frequency, self.expected_version, self.timeout_s)
            if not self.host_clock_id:
                raise protocol.ProtocolError('host boot/monotonic clock identity unavailable')
            self._controller_version, self._recipe_id = version, recipe
            self._state('connected', 'output recipe accepted; waiting for first packet')
            # One absolute deadline per data packet: a stream of RTDE text
            # messages must not keep old measurements marked streaming.
            data_deadline = time.monotonic() + self.timeout_s
            while not self._stop.is_set():
                kind, data, host_ns = protocol.read_packet(sock, data_deadline, self.timeout_s)
                if kind == ord('M'):
                    continue
                if kind != ord('U'):
                    raise protocol.ProtocolError('unexpected RTDE output packet')
                sample = protocol.decode(data, recipe)
                self._accept(sample, host_ns)
                self._state('streaming')
                data_deadline = host_ns / 1e9 + self.timeout_s
        except protocol.CaptureDeadline:
            if not self._stop.is_set():
                with self._lock:
                    self._stats['transport_errors'] += 1
                self._state('fault', 'RTDE data-packet deadline expired; session ended')
        except protocol.PacketDeadline:
            if not self._stop.is_set():
                with self._lock:
                    self._stats['transport_errors'] += 1
                    self._stats['malformed_packets'] += 1
                self._state('fault', 'RTDE data-packet deadline expired inside a frame; session ended')
        except protocol.TransportError as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._stats['transport_errors'] += 1
                self._state('disconnected', str(exc))
        except (protocol.CompatibilityError, TimestampRegression) as exc:
            if not self._stop.is_set():
                self._state('fault', str(exc))
        except protocol.ProtocolError as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._stats['malformed_packets'] += 1
                self._state('fault', str(exc))
        except OSError as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._stats['transport_errors'] += 1
                self._state('disconnected', str(exc))
        finally:
            if self._stop.is_set():
                self._state('stopped')

    def run(self):
        """Connect once. Only explicitly authorized collector launch calls this."""
        if self._stop.is_set():
            self._state('stopped')
            return
        try:
            sock = self._connect((self.host, 30004), timeout=min(3.0, self.timeout_s))
            with self._lock:
                self._socket = sock
            try:
                if not self._stop.is_set():
                    self.receive_from(sock)
            finally:
                sock.close()
        except OSError as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._stats['transport_errors'] += 1
                self._state('disconnected', str(exc))
        finally:
            with self._lock:
                self._socket = None
            if self._stop.is_set():
                self._state('stopped')

    def stop(self):
        self._stop.set()
        with self._lock:
            sock = self._socket
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def publish_result(self, success):
        with self._lock:
            if success:
                self._stats['packets_published'] += 1
            else:
                self._stats['publication_failures'] += 1
