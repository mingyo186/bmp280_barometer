#!/usr/bin/env python3
"""ROS2 node that reads BMP280 over I2C and publishes pressure + temperature."""

import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import FluidPressure, Temperature
from std_srvs.srv import Trigger

from bmp280_barometer.bmp280_driver import BMP280Driver, FakeBMP280Driver


class BMP280BarometerNode(Node):
    def __init__(self):
        super().__init__('bmp280_barometer_node')

        # ── Declare parameters ────────────────────────────────────
        self.declare_parameter('fake_mode', True)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('device_address', 0x76)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('frame_id', 'bmp280_link')
        self.declare_parameter('pressure_oversampling', 5)
        self.declare_parameter('temperature_oversampling', 2)
        self.declare_parameter('iir_filter', 4)
        self.declare_parameter('pressure_variance', 0.0)
        self.declare_parameter('temperature_variance', 0.0)

        # ── Read parameters ───────────────────────────────────────
        self.fake_mode = self.get_parameter('fake_mode').value
        self.bus_num   = self.get_parameter('i2c_bus').value
        self.address   = self.get_parameter('device_address').value
        rate           = self.get_parameter('publish_rate').value
        self.frame_id  = self.get_parameter('frame_id').value
        self.press_os  = self.get_parameter('pressure_oversampling').value
        self.temp_os   = self.get_parameter('temperature_oversampling').value
        self.iir_filter = self.get_parameter('iir_filter').value
        self.press_var = self.get_parameter('pressure_variance').value
        self.temp_var  = self.get_parameter('temperature_variance').value

        # ── Pressure / temperature bias (set by calibration) ──────
        self.press_bias = 0.0
        self.temp_bias = 0.0

        # ── Initialise driver ─────────────────────────────────────
        self._init_driver()

        # ── Publishers + timer ────────────────────────────────────
        self.pub_press = self.create_publisher(
            FluidPressure, 'bmp280/pressure', 10)
        self.pub_temp = self.create_publisher(
            Temperature, 'bmp280/temperature', 10)
        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(
            f'Publishing on "bmp280/pressure" and "bmp280/temperature" '
            f'@ {rate} Hz')

        # ── Services ──────────────────────────────────────────────
        self.create_service(Trigger, 'bmp280/calibrate', self._calibrate_cb)
        self.create_service(Trigger, 'bmp280/reset', self._reset_cb)
        self.get_logger().info(
            'Services: "bmp280/calibrate", "bmp280/reset"')

        # ── Parameter change callback ─────────────────────────────
        self.add_on_set_parameters_callback(self._on_param_change)

    # ── Driver init helper ───────────────────────────────────────
    def _init_driver(self):
        if self.fake_mode:
            self.driver = FakeBMP280Driver()
            self.get_logger().info(
                'FAKE MODE enabled — generating random pressure/temperature data')
        else:
            try:
                self.driver = BMP280Driver(
                    self.bus_num, self.address,
                    self.press_os, self.temp_os, self.iir_filter)
                cid = self.driver.chip_id()
                self.get_logger().info(
                    f'BMP280 initialised  bus={self.bus_num}  '
                    f'addr=0x{self.address:02X}  CHIP_ID=0x{cid:02X}')
            except Exception as e:
                self.get_logger().fatal(f'Failed to open BMP280: {e}')
                raise

    # ── Timer callback ───────────────────────────────────────────
    def _timer_cb(self):
        try:
            pressure, temperature = self.driver.read_all()
        except OSError as e:
            self.get_logger().warn(
                f'I2C read error: {e}', throttle_duration_sec=2.0)
            return

        # Apply bias correction
        pressure -= self.press_bias
        temperature -= self.temp_bias

        stamp = self.get_clock().now().to_msg()

        # ── FluidPressure (Pa) ────────────────────────────────────
        press_msg = FluidPressure()
        press_msg.header.stamp = stamp
        press_msg.header.frame_id = self.frame_id
        press_msg.fluid_pressure = pressure
        press_msg.variance = self.press_var
        self.pub_press.publish(press_msg)

        # ── Temperature (°C) ─────────────────────────────────────
        temp_msg = Temperature()
        temp_msg.header.stamp = stamp
        temp_msg.header.frame_id = self.frame_id
        temp_msg.temperature = temperature
        temp_msg.variance = self.temp_var
        self.pub_temp.publish(temp_msg)

    # ── Service: /bmp280/calibrate ───────────────────────────────
    def _calibrate_cb(self, request, response):
        if self.fake_mode:
            response.success = True
            response.message = 'Calibration complete (fake)'
            self.get_logger().info('Calibration requested in fake mode — skipped')
            return response

        self.get_logger().info('Calibrating — collecting data for 2 seconds...')
        press_samples = []
        temp_samples = []
        end_time = time.monotonic() + 2.0
        while time.monotonic() < end_time:
            try:
                p, t = self.driver.read_all()
                press_samples.append(p)
                temp_samples.append(t)
            except OSError:
                pass
            time.sleep(0.05)

        if not press_samples:
            response.success = False
            response.message = 'Calibration failed — no samples collected'
            return response

        n = len(press_samples)
        self.press_bias = sum(press_samples) / n - 101325.0  # offset from std atm
        self.temp_bias = 0.0  # temperature bias is usually not corrected

        response.success = True
        response.message = (
            f'Calibration complete — {n} samples, '
            f'pressure_bias={self.press_bias:.2f} Pa')
        self.get_logger().info(response.message)
        return response

    # ── Service: /bmp280/reset ───────────────────────────────────
    def _reset_cb(self, request, response):
        self.press_bias = 0.0
        self.temp_bias = 0.0
        self.driver.close()
        self._init_driver()

        response.success = True
        response.message = 'Sensor reset complete'
        self.get_logger().info(response.message)
        return response

    # ── Runtime parameter change ─────────────────────────────────
    def _on_param_change(self, params):
        for param in params:
            if param.name == 'publish_rate':
                new_rate = param.value
                if new_rate <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='publish_rate must be > 0')
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / new_rate, self._timer_cb)
                self.get_logger().info(f'publish_rate changed to {new_rate} Hz')
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = BMP280BarometerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
