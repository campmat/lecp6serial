# LECP6Serial Wrapper

A Python library for controlling SMC LECP6 actuators via serial communication.

## Features

- Enable, Reset, Home, and Start operations
- Send data for position, speed, and other configurations
- Monitor input and output signals
- Retrieve the current actuator position
- Monitor position during movement with logging capabilities

## Installation

Clone the repository and install the package:

```bash
git clone https://github.com/campmat/lecp6serial.git
cd lecp6serial
pip install .
```

## Usage

### Quickstart

```python
from lecp6serial import LECP6Serial

lecp6 = LECP6Serial(port="COM4")
lecp6.move_to(position=150) # move the actuator to 150 mm
```

### LECP6Serial Object Parameters:


|Parameter |Type   |Default |Description                                 |
|----------|-------|--------|--------------------------------------------|
|`port`    |`str`  |Required|The serial port to connect to (e.g., "COM4")|
|`baudrate`|`int`  |`38400` |Baud rate for the serial connection.        |
|`timeout` |`float`|`0.1`   |Timeout in seconds for serial communication.|
|`CTRL_ID` |`int`  |`1`     |Controller ID for the actuator.             |


### Key functions

`move_to`

Moves actuator to a target position. Parameters:

| Parameter         | Type    | Default | Description                                                                              |
|-------------------|---------|---------|------------------------------------------------------------------------------------------|
| `movement_mode`   | `int`   | `1`     | Movement mode: `1` for absolute, `2` for relative movement.                              |
| `speed`           | `int`   | `16`    | Speed in mm/s (`16-500`).                                                                |
| `position`        | `float` | `0.00`  | Target position in mm.                                                                   |
| `acceleration`    | `int`   | `3000`  | Acceleration in mm/s².                                                                   |
| `deceleration`    | `int`   | `3000`  | Deceleration in mm/s².                                                                   |
| `pushing_force`   | `int`   | `0`     | Pushing force as a percentage (`0-100`). A value of `0` indicates positioning operation. |
| `trigger_level`   | `int`   | `0`     | Trigger level as a percentage (`0-100`).                                                 |
| `pushing_speed`   | `int`   | `16`    | Pushing speed in mm/s.                                                                   |
| `moving_force`    | `int`   | `100`   | Moving force as a percentage (`0-300`).                                                  |
| `area1`           | `float` | `0.00`  | Area output end 1 in mm.                                                                 |
| `area2`           | `float` | `0.00`  | Area output end 2 in mm.                                                                 |
| `in_position`     | `float` | `0.00`  | In-position tolerance in mm.                                                             |
| `log`             | `bool`  | `False` | Enable position logging during movement.                                                 |
| `pre_log_points`  | `int`   | `5`     | Number of points to log before movement starts.                                          |
| `polling_interval`| `float` | `0.01`  | Time interval (in seconds) between position readings.                                    |

`get_current_position`

Retrieves the current position of the actuator in millimeters.

```python
current_position = lecp6.get_current_position()
print(f"Current Position: {current_position} mm")
```

`monitor_position_during_movement`

Monitors the actuator's position during movement. This function is called internally by `move_to` when `log=True`. It returns a log of `(timestamp, position, time_difference)` tuples.


```python
position_log = lecp6.move_to(position=200, log=True)
for entry in position_log:
    print(f"Time: {entry[0]} | Position: {entry[1]} mm | Interval: {entry[2]} seconds")
```

`close`

Turns off the servo and closes the serial connection. Called in destructor.

```python
lecp6.close()
```
