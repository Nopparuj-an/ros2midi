# ros2midi

A ROS2 package to connect your MIDI devices to ROS.

## Tested Version
- Ubuntu 22.04
- ROS2 Humble

## Dependencies
- Python 3 + rclpy
- [mido](https://pypi.org/project/mido/) Python library

## Installation & Build
1. Clone this repository into `src` of your ROS2 workspace.
2. From the workspace root, run: `colcon build --packages-select ros2midi`
3. Source the workspace: `source install/setup.bash`

## Usage
After building, run each script (e.g. `piano_simple.py`) with:
```bash
ros2 run ros2midi piano_simple.py
```

## Topics
- `/midi/receive` (ros2midi/msg/PianoSimple)
    
    Inbound MIDI messages are published to this topic.

- `/midi/transmit` (ros2midi/msg/PianoSimple)

    Messages published to this topic are sent to the MIDI output.

## Message Structure (ros2midi/msg/PianoSimple)
```
string type     # "note_on", "note_off", "control_change"
uint8 channel   # 0-15
uint8 note      # 0-127
uint8 velocity  # 0-127
uint8 control   # 0-127 (64 for sustain pedal)
uint8 value     # 0-127
```

1. **Note On Event**
    - **type**: "note_on"
    - **channel**: MIDI channel (default: 0)
    - **note**: MIDI note number (0-127)
    - **velocity**: MIDI velocity (0-127)
    - **control & value**: 0 (unused)
2. **Note Off Event**
    - **type**: "note_off"
    - **channel**: MIDI channel (default: 0)
    - **note**: MIDI note number (0-127)
    - **velocity**: MIDI velocity (0-127)
    - **control & value**: 0 (unused)
3. **Control Change Event**
    - **type**: "control_change"
    - **channel**: MIDI channel (default: 0)
    - **control**: MIDI control number (0-127)
    - **value**: MIDI control value (0-127)
    - **note & velocity**: 0 (unused)

## License
MIT License. See [LICENSE](LICENSE) file for more details.

## Contributing
Contributions are welcome! Please open an issue or submit a pull request for any changes.