# bmp280_barometer

ROS 2 Jazzy driver for the Bosch BMP280 barometric pressure and temperature sensor over I2C.

## Features

- Publishes `sensor_msgs/FluidPressure` on the `bmp280/pressure` topic
- Publishes `sensor_msgs/Temperature` on the `bmp280/temperature` topic
- **Fake mode** — generates random pressure/temperature data without physical hardware
- Configurable I2C bus, device address, publish rate, oversampling, and IIR filter
- Double-precision compensation formulas from the BMP280 datasheet (Section 8.1)
- Burst-read of pressure + temperature in a single I2C transaction

## Prerequisites

- ROS 2 Jazzy
- Python 3
- `smbus2` (only required when `fake_mode` is `false`)

```bash
pip3 install smbus2
```

## Installation

```bash
cd ~/ros2_ws
colcon build --packages-select bmp280_barometer
source install/setup.bash
```

## Usage

### Launch with default parameters (fake mode)

```bash
ros2 launch bmp280_barometer bmp280_launch.py
```

### Run the node directly

```bash
ros2 run bmp280_barometer bmp280_node.py --ros-args -p fake_mode:=true
```

### Run with real hardware

```bash
ros2 run bmp280_barometer bmp280_node.py --ros-args -p fake_mode:=false
```

### Override parameters via YAML

```bash
ros2 launch bmp280_barometer bmp280_launch.py params_file:=/path/to/your_params.yaml
```

### Verify output

```bash
ros2 topic echo /bmp280/pressure
ros2 topic echo /bmp280/temperature
```

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `fake_mode` | bool | `true` | `true`: generate random data, `false`: read from real I2C device |
| `i2c_bus` | int | `1` | I2C bus number (`/dev/i2c-N`) |
| `device_address` | int | `0x76` | BMP280 I2C address (`0x76` or `0x77`) |
| `publish_rate` | double | `10.0` | Publishing rate in Hz |
| `frame_id` | string | `bmp280_link` | TF frame ID in message headers |
| `pressure_oversampling` | int | `5` | Pressure oversampling: `0`=skip, `1`=x1, `2`=x2, `3`=x4, `4`=x8, `5`=x16 |
| `temperature_oversampling` | int | `2` | Temperature oversampling: `0`=skip, `1`=x1, `2`=x2, `3`=x4, `4`=x8, `5`=x16 |
| `iir_filter` | int | `4` | IIR filter coefficient: `0`=off, `1`=2, `2`=4, `3`=8, `4`=16 |
| `pressure_variance` | double | `0.0` | Pressure variance (0 = unknown) |
| `temperature_variance` | double | `0.0` | Temperature variance (0 = unknown) |

## Services

| Service | Type | Description |
|---|---|---|
| `bmp280/calibrate` | `std_srvs/Trigger` | Collect data for 2 seconds and compute pressure bias to standard atmosphere |
| `bmp280/reset` | `std_srvs/Trigger` | Clear bias and reinitialize the sensor |

## Package Structure

```
bmp280_barometer/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── bmp280_params.yaml
├── launch/
│   └── bmp280_launch.py
├── bmp280_barometer/
│   ├── __init__.py
│   └── bmp280_driver.py
├── nodes/
│   └── bmp280_node.py
└── test/
    └── test_bmp280_node.py
```

## Test Results

Tested on Ubuntu 24.04 (WSL2) with `fake_mode: true`.

```
$ colcon test --packages-select bmp280_barometer
$ colcon test-result --verbose
Summary: 28 tests, 0 errors, 0 failures, 0 skipped
```

| Test Category | Test | Result |
|---|---|---|
| **Topics** | `bmp280/pressure` publishes `sensor_msgs/FluidPressure` | PASS |
| **Topics** | `bmp280/temperature` publishes `sensor_msgs/Temperature` | PASS |
| **Topics** | `frame_id == "bmp280_link"` | PASS |
| **Topics** | Pressure in range 90000-110000 Pa | PASS |
| **Topics** | Temperature in range 15-35 °C | PASS |
| **Services** | `bmp280/calibrate` returns `success=True` | PASS |
| **Services** | `bmp280/reset` returns `success=True` | PASS |
| **Parameters** | `publish_rate` runtime change to 20.0 Hz | PASS |
| **Shutdown** | Clean exit (code 0, -2, or -15) | PASS |
| **Linting** | pep257, flake8, copyright, xmllint | PASS |

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
