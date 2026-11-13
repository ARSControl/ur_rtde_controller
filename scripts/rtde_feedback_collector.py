#!/usr/bin/env python3
"""Separate output-only RTDE collector; does not drive or configure a robot."""
import queue
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from ur_rtde_controller.msg import RtdeFeedbackHealth, RtdeFeedbackPacket

from ur_rtde_feedback.collector import FeedbackCollector


class RtdeFeedbackCollector(Node):
    def __init__(self, collector_factory=FeedbackCollector):
        super().__init__('rtde_feedback_collector')
        self.declare_parameter('robot_ip', '192.168.2.40')
        self.declare_parameter('frequency', 500.0)
        self.declare_parameter('queue_capacity', 512)
        self.declare_parameter('expected_version', '5.9.3')
        self.declare_parameter('timeout_s', 3.0)
        self.collector = collector_factory(self.get_parameter('robot_ip').value,
                                           self.get_parameter('frequency').value,
                                           self.get_parameter('queue_capacity').value,
                                           self.get_parameter('expected_version').value,
                                           timeout_s=self.get_parameter('timeout_s').value)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=64,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.packet_publisher = self.create_publisher(RtdeFeedbackPacket,
            '/ur_rtde/diagnostics/rtde_feedback_packet', qos)
        self.health_publisher = self.create_publisher(RtdeFeedbackHealth,
            '/ur_rtde/diagnostics/rtde_feedback_health', qos)
        # Publish initial counters before socket ingestion. Best-effort DDS
        # delivery is not guaranteed; absence of a recorded bracket makes the
        # affected analysis incomplete, never loss-free by assumption.
        self.publish_health()
        self.publisher_stop = threading.Event()
        self.receiver = threading.Thread(target=self.collector.run, name='rtde-output-reader', daemon=True)
        self.sender = threading.Thread(target=self._publish_packets, name='rtde-evidence-publisher', daemon=True)
        self.sender.start()
        self.receiver.start()
        self.create_timer(1.0, self.publish_health)

    def _publish_packets(self):
        while not self.publisher_stop.is_set() or not self.collector.queue.empty():
            try:
                packet = self.collector.queue.get(timeout=0.1)
            except queue.Empty:
                continue
            success = False
            try:
                message = RtdeFeedbackPacket()
                for key, value in packet.items():
                    setattr(message, key, value)
                self.packet_publisher.publish(message)
                success = True
            except Exception as exc:  # failure loses evidence; never retries or affects control
                self.get_logger().error(f'Feedback publication failed: {exc}')
            finally:
                self.collector.publish_result(success)
                self.collector.queue.task_done()

    def publish_health(self):
        message = RtdeFeedbackHealth()
        for key, value in self.collector.health().items():
            setattr(message, key, value)
        self.health_publisher.publish(message)

    def close_collector(self):
        self.collector.stop()
        self.receiver.join(timeout=4.0)
        self.publisher_stop.set()
        self.sender.join(timeout=1.0)
        # A stuck reader/publisher means final counters cannot be assumed
        # complete; never block the unrelated control process on diagnostics.
        if self.receiver.is_alive() or self.sender.is_alive():
            self.get_logger().error('Collector worker did not stop within timeout; final coverage unknown')
            self.collector._state('fault', 'worker shutdown timed out; final counters incomplete')
        if rclpy.ok():
            self.publish_health()
            # Allow a bounded opportunity for best-effort delivery before
            # destroying the publisher; delivery is still not guaranteed.
            time.sleep(0.1)


def main(args=None):
    # Keep ROS context alive through the final health publication when this
    # independent collector receives SIGINT/SIGTERM from the recorder.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = threading.Event()
    previous = {}
    def request_stop(_signum, _frame):
        stop_requested.set()
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous[signum] = signal.signal(signum, request_stop)
    node = None
    try:
        node = RtdeFeedbackCollector()
        while rclpy.ok() and not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        if node is not None:
            node.close_collector()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    main()
