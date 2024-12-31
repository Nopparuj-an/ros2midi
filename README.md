# ros2midi

A ROS2 package to connect your MIDI devices to ROS.

## Tested Version
- Ubuntu 22.04
- ROS2 Humble

## Dependencies
- Python 3
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

## Message Structure (ros2midi/msg/PianoSimple)
- `string type`
- `uint8 channel`
- `uint8 note`
- `uint8 velocity`
- `uint8 control`
- `uint8 value`

## License
MIT License. See [LICENSE](LICENSE) file for more details.
