#!/usr/bin/env python3
"""Capture an output-only RTDE telemetry stream; does not command the robot.

RTDE version, controller version, output setup and start are protocol requests
sent to port 30004; no input recipe, input data, URScript or motion command is
sent. This is an independent client and may add robot/network load. It does not
establish command identity, robot acknowledgement, or physical latency.
"""
import argparse
import json
import math
import socket
import struct
import sys
import time
from pathlib import Path


FIELDS = (
    ('timestamp', 'DOUBLE'), ('actual_q', 'VECTOR6D'), ('actual_qd', 'VECTOR6D'),
    ('target_qd', 'VECTOR6D'), ('actual_TCP_pose', 'VECTOR6D'),
    ('actual_TCP_speed', 'VECTOR6D'), ('speed_scaling', 'DOUBLE'),
    ('robot_status_bits', 'UINT32'), ('safety_status_bits', 'UINT32'),
)
TYPES = ','.join(kind for _, kind in FIELDS)
DATA = struct.Struct('!d' + '6d' * 5 + 'dII')
MAX_PACKET = 65535
REQUESTS = {ord('V'), ord('v'), ord('O'), ord('S')}


class ProtocolError(Exception):
    pass


class TransportError(ProtocolError):
    """Socket closed or timed out; distinct from a malformed RTDE frame."""


class CompatibilityError(ProtocolError):
    """The controller version or supported output recipe is incompatible."""


class CaptureDeadline(Exception):
    """Deadline reached between packets, with no partial frame."""


class PacketDeadline(ProtocolError):
    """Deadline reached while a packet was only partially read."""


def send(sock, command, payload=b''):
    if command not in REQUESTS:
        raise ProtocolError('only version, output setup and output start requests are allowed')
    sock.sendall(struct.pack('!HB', 3 + len(payload), command) + payload)


def read_exact(sock, n, deadline=None, allow_empty_deadline=False, io_timeout_s=3.0):
    out = bytearray()
    while len(out) < n:
        if deadline is not None:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                if allow_empty_deadline and not out:
                    raise CaptureDeadline()
                raise PacketDeadline('capture ended inside an RTDE packet')
            sock.settimeout(min(io_timeout_s, remaining))
        try:
            block = sock.recv(n - len(out))
        except socket.timeout as exc:
            if deadline is not None and time.monotonic() >= deadline:
                if allow_empty_deadline and not out:
                    raise CaptureDeadline() from exc
                raise PacketDeadline('capture ended inside an RTDE packet') from exc
            raise TransportError('timeout while receiving RTDE packet') from exc
        if not block:
            raise TransportError('RTDE socket closed mid-stream')
        out.extend(block)
    return bytes(out)


def read_packet(sock, deadline=None, io_timeout_s=3.0):
    size, kind = struct.unpack('!HB', read_exact(sock, 3, deadline, allow_empty_deadline=True,
                                                io_timeout_s=io_timeout_s))
    if size < 3 or size > MAX_PACKET:
        raise ProtocolError(f'invalid RTDE packet size {size}')
    data = read_exact(sock, size - 3, deadline, io_timeout_s=io_timeout_s)
    # Completion time of the socket read, not kernel/network or robot sample time.
    return kind, data, time.monotonic_ns()


def response(sock, wanted):
    # An RTDE_TEXT_MESSAGE can interleave protocol responses; bounded so a
    # faulty peer cannot trap startup indefinitely in text-message handling.
    for _ in range(32):
        kind, data, _ = read_packet(sock)
        if kind == wanted:
            return data
        if kind != ord('M'):
            raise ProtocolError(f'unexpected packet type {kind} waiting for {wanted}')
    raise ProtocolError('too many text messages while waiting for response')


def require_accepted(data, stage):
    if data != b'\x01':
        raise ProtocolError(f'{stage} rejected or malformed')


def setup(sock, frequency, expected_version=None, timeout_s=3.0):
    sock.settimeout(timeout_s)
    send(sock, ord('V'), struct.pack('!H', 2))
    require_accepted(response(sock, ord('V')), 'protocol v2')
    send(sock, ord('v'))
    version_data = response(sock, ord('v'))
    if len(version_data) != 16:
        raise ProtocolError('invalid controller version response')
    version = '.'.join(str(i) for i in struct.unpack('!IIII', version_data))
    if expected_version is not None and version.split('.')[:3] != expected_version.split('.')[:3]:
        raise CompatibilityError('RTDE controller version differs from expected version')
    send(sock, ord('O'), struct.pack('!d', frequency) + ','.join(name for name, _ in FIELDS).encode('ascii'))
    setup_data = response(sock, ord('O'))
    if not setup_data or setup_data[0] == 0:
        raise CompatibilityError('output recipe rejected; robot may lack a requested field')
    try:
        types = setup_data[1:].decode('ascii')
    except UnicodeDecodeError as exc:
        raise ProtocolError('invalid output recipe response') from exc
    if types != TYPES:
        raise CompatibilityError(f'incompatible output types: {types!r}')
    send(sock, ord('S'))
    require_accepted(response(sock, ord('S')), 'stream start')
    return version, setup_data[0]


def decode(data, recipe):
    if len(data) != DATA.size + 1 or data[0] != recipe:
        raise ProtocolError('wrong recipe id or truncated/oversized output sample')
    values = DATA.unpack(data[1:])
    if not all(math.isfinite(x) for x in values):
        raise ProtocolError('non-finite RTDE sample')
    return {'timestamp': values[0], 'actual_q': values[1:7], 'actual_qd': values[7:13],
            'target_qd': values[13:19], 'actual_TCP_pose': values[19:25],
            'actual_TCP_speed': values[25:31], 'speed_scaling': values[31],
            'robot_status_bits': values[32], 'safety_status_bits': values[33]}


def emit(out, row):
    out.write(json.dumps(row, separators=(',', ':'), allow_nan=False) + '\n')


def host_clock_id():
    try:
        boot_id = Path('/proc/sys/kernel/random/boot_id').read_text(encoding='ascii').strip()
    except OSError:
        return None
    return f'{socket.gethostname()}:{boot_id}:CLOCK_MONOTONIC' if boot_id else None


def capture(sock, out, *, duration, frequency, peer):
    if not math.isfinite(duration) or not 0 < duration <= 300:
        raise ValueError('duration must be greater than 0 and at most 300 seconds')
    if not math.isfinite(frequency) or not 1 <= frequency <= 500:
        raise ValueError('frequency must be between 1 and 500 Hz')
    count = gaps = nonmonotonic = 0
    last_robot = None
    version, recipe = setup(sock, frequency)
    begin = time.monotonic()
    emit(out, {'record': 'metadata', 'schema': 1, 'peer': peer,
               'controller_version': version, 'requested_frequency_hz': frequency,
               'duration_seconds': duration, 'fields': FIELDS,
               'host_clock': 'CLOCK_MONOTONIC', 'host_clock_id': host_clock_id(),
               'robot_clock': 'controller uptime seconds',
               'limits': 'local sequence is not a robot packet sequence; no command identity or clock alignment'})
    try:
        while True:
            try:
                kind, payload, host_ns = read_packet(sock, begin + duration)
            except CaptureDeadline:
                break
            if kind == ord('M'):
                continue
            if kind != ord('U'):
                raise ProtocolError(f'unexpected streaming packet type {kind}')
            sample = decode(payload, recipe)
            robot = sample['timestamp']
            if last_robot is not None:
                delta = robot - last_robot
                if delta <= 0:
                    nonmonotonic += 1
                # A long interval is an observation, not an inferred drop count.
                if delta > 1.5 / frequency:
                    gaps += 1
            last_robot = robot
            count += 1
            emit(out, {'record': 'sample', 'local_sequence': count,
                       'host_packet_read_complete_ns': host_ns, **sample})
        status = 'complete' if count else 'error_no_samples'
    except (ProtocolError, OSError, ValueError) as exc:
        emit(out, {'record': 'summary', 'status': 'error', 'error': str(exc),
                   'samples': count, 'long_robot_intervals': gaps, 'nonmonotonic_robot_timestamps': nonmonotonic})
        raise
    emit(out, {'record': 'summary', 'status': status, 'samples': count,
               'long_robot_intervals': gaps, 'nonmonotonic_robot_timestamps': nonmonotonic,
               'gap_note': 'threshold 1.5/requested frequency; not a packet loss count'})
    if not count:
        raise ProtocolError('capture ended without samples')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True, help='robot IP address or hostname')
    parser.add_argument('--output', type=Path, required=True, help='new JSONL file')
    parser.add_argument('--duration', type=float, default=10.0, help='seconds (0 < duration <= 300)')
    parser.add_argument('--frequency', type=float, default=125.0, help='requested output Hz (1..500)')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 300 or not math.isfinite(args.frequency) or not 1 <= args.frequency <= 500:
        parser.error('duration must be 0 < duration <= 300, frequency 1..500')
    try:
        with args.output.open('x', encoding='utf-8') as out:
            # RTDE only: never create RTDEControlInterface or a dashboard client.
            with socket.create_connection((args.host, 30004), timeout=3.0) as sock:
                capture(sock, out, duration=args.duration, frequency=args.frequency, peer=args.host)
    except (OSError, ProtocolError, ValueError) as exc:
        print(f'Capture failed: {exc}', file=sys.stderr)
        return 1
    print(f'Output-only RTDE capture saved to {args.output}; inspect the final summary.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
