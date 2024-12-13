# LECP6Serial Wrapper

A Python library for controlling SMC LECP6 actuators via serial communication.

## Features

- Enable, Reset, Home, and Start operations
- Send data for position, speed, and other configurations
- Monitor input and output signals

## Installation

Clone the repository and install the package:

```bash
git clone https://github.com/campmat/lecp6serial.git
cd lecp6serial
pip install .
```

## Usage

```python
from lecp6serial import LECP6Serial

lecp6 = LECP6Serial(port="COM4")
lecp6.move_to(position=150)

```

| Parameter        | Type    | Default | Description                                                                              |
|------------------|---------|---------|------------------------------------------------------------------------------------------|
| `movement_mode`  | `int`   | `1`     | Movement mode: `1` for absolute, `2` for relative movement.                              |
| `speed`          | `int`   | `16`    | Speed in mm/s (`16-500`).                                                                |
| `position`       | `float` | `0.00`  | Target position in mm.                                                                   |
| `acceleration`   | `int`   | `3000`  | Acceleration in mm/s².                                                                   |
| `deceleration`   | `int`   | `3000`  | Deceleration in mm/s².                                                                   |
| `pushing_force`  | `int`   | `0`     | Pushing force as a percentage (`0-100`). A value of `0` indicates positioning operation. |
| `trigger_level`  | `int`   | `0`     | Trigger level as a percentage (`0-100`).                                                 |
| `pushing_speed`  | `int`   | `16`    | Pushing speed in mm/s.                                                                   |
| `moving_force`   | `int`   | `100`   | Moving force as a percentage (`0-300`).                                                  |
| `area1`          | `float` | `0.00`  | Area output end 1 in mm.                                                                 |
| `area2`          | `float` | `0.00`  | Area output end 2 in mm.                                                                 |
| `in_position`    | `float` | `0.00`  | In-position tolerance in mm.                                                             |
