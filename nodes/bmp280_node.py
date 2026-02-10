#!/usr/bin/env python3
"""ROS2 node that reads BMP280 over I2C and publishes pressure + temperature."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Temperature

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
        fake_mode  = self.get_parameter('fake_mode').value
        bus        = self.get_parameter('i2c_bus').value
        address    = self.get_parameter('device_address').value
        rate       = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        press_os   = self.get_parameter('pressure_oversampling').value
        temp_os    = self.get_parameter('temperature_oversampling').value
        iir_filter = self.get_parameter('iir_filter').value
        self.press_var = self.get_parameter('pressure_variance').value
        self.temp_var  = self.get_parameter('temperature_variance').value

        # ── Initialise driver ─────────────────────────────────────
        if fake_mode:
            self.driver = FakeBMP280Driver()
            self.get_logger().info(
                'FAKE MODE enabled — generating random pressure/temperature data')
        else:
            try:
                self.driver = BMP280Driver(
                    bus, address, press_os, temp_os, iir_filter)
                cid = self.driver.chip_id()
                self.get_logger().info(
                    f'BMP280 initialised  bus={bus}  addr=0x{address:02X}  '
                    f'CHIP_ID=0x{cid:02X}')
            except Exception as e:
                self.get_logger().fatal(f'Failed to open BMP280: {e}')
                raise

        # ── Publishers + timer ────────────────────────────────────
        self.pub_press = self.create_publisher(
            FluidPressure, 'bmp280/pressure', 10)
        self.pub_temp = self.create_publisher(
            Temperature, 'bmp280/temperature', 10)
        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(
            f'Publishing on "bmp280/pressure" and "bmp280/temperature" '
            f'@ {rate} Hz')

    # ──────────────────────────────────────────────────────────────
    def _timer_cb(self):
        try:
            pressure, temperature = self.driver.read_all()
        except OSError as e:
            self.get_logger().warn(
                f'I2C read error: {e}', throttle_duration_sec=2.0)
            return

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
