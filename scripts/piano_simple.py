#!/usr/bin/python3

import mido
import rclpy
from rclpy.node import Node
from ros2midi.msg import PianoSimple


class PianoSimpleNode(Node):
    def __init__(self):
        super().__init__("piano_simple_node")
        self.logger = self.get_logger()
        # self.logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Initialize ROS2 Node
        self.publisher = self.create_publisher(PianoSimple, "midi/receive", 10)
        self.subscriber = self.create_subscription(PianoSimple, "midi/transmit", self.midi_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Initialize MIDI
        inputs = self.filter_midi_devices(mido.get_input_names())
        if len(inputs) == 0:
            self.logger.warn("No MIDI input devices found")
        else:
            self.logger.info(f"Found MIDI input devices: {inputs}")
            input = inputs[0]
            self.logger.info(f"Using MIDI input device: {input}")
            self.midi_in = mido.open_input(input)
        
        outputs = self.filter_midi_devices(mido.get_output_names())
        if len(outputs) == 0:
            self.logger.warn("No MIDI output devices found")
        else:
            self.logger.info(f"Found MIDI output devices: {outputs}")
            output = outputs[0]
            self.logger.info(f"Using MIDI output device: {output}")
            self.midi_out = mido.open_output(output)

    def midi_callback(self, msg):
        type = msg.type
        channel = msg.channel
        note = msg.note
        velocity = msg.velocity
        control = msg.control
        value = msg.value
        self.logger.debug(f"Received: {type} CH:{channel} KEY:{note} VEL:{velocity} CTRL:{control} VAL:{value}")

        # Send MIDI message
        if type == "note_on":
            self.midi_out.send(mido.Message("note_on", channel=channel, note=note, velocity=velocity))
        elif type == "note_off":
            self.midi_out.send(mido.Message("note_off", channel=channel, note=note, velocity=velocity))
        elif type == "control_change":
            self.midi_out.send(mido.Message("control_change", channel=channel, control=control, value=value))
        else:
            self.logger.debug(f"Unsupported MIDI message: {msg}")

    def timer_callback(self):
        for msg in self.midi_in.iter_pending():
            self.logger.debug(f"MIDI Input: {msg}")

            # Default values
            channel = 0
            note = 0
            velocity = 0
            control = 0
            value = 0

            # Parse MIDI message
            if msg.type == "note_on":
                channel = msg.channel
                note = msg.note
                velocity = msg.velocity
            elif msg.type == "note_off":
                channel = msg.channel
                note = msg.note
                velocity = msg.velocity
            elif msg.type == "control_change":
                channel = msg.channel
                control = msg.control
                value = msg.value
            else:
                self.logger.debug(f"Unsupported MIDI message: {msg}")
                continue

            # Publish to ROS2
            out_msg = PianoSimple()
            out_msg.type = msg.type
            out_msg.channel = channel
            out_msg.note = note
            out_msg.velocity = velocity
            out_msg.control = control
            out_msg.value = value
            self.publisher.publish(out_msg)

    def filter_midi_devices(self, devices):
        return [device for device in devices if ("Midi Through" not in str(device)) and ("RtMidiIn Client" not in str(device))]


def main(args=None):
    rclpy.init(args=args)
    node = PianoSimpleNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        # Send MIDI all note off messages
        node.midi_out.panic()


if __name__ == "__main__":
    main()
