#!/usr/bin/python3

import time
import threading
import rclpy
from rclpy.node import Node
from ros2midi.msg import PianoSimple


class MidiDelayNode(Node):
    def __init__(self, delay_seconds):
        super().__init__("midi_delay_node")
        self.logger = self.get_logger()
        self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.delay_seconds = delay_seconds

        # Initialize ROS2 Node
        self.subscriber = self.create_subscription(PianoSimple, "midi/receive", self.midi_callback, 10)
        self.publisher = self.create_publisher(PianoSimple, "midi/transmit", 10)

    def midi_callback(self, msg):
        self.logger.debug(f"Received MIDI message: {msg}")
        # Delay the message by X seconds
        threading.Thread(target=self.delayed_publish, args=(msg,)).start()

    def delayed_publish(self, msg):
        time.sleep(self.delay_seconds)
        self.publisher.publish(msg)
        self.logger.debug(f"Published MIDI message after {self.delay_seconds} seconds delay: {msg}")

def main(args=None):
    rclpy.init(args=args)
    delay_seconds = 2  # Set your desired delay here
    node = MidiDelayNode(delay_seconds)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()