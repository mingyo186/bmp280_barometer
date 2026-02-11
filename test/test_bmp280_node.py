# Copyright 2025 The bmp280_barometer Authors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""Launch test for BMP280 barometric pressure sensor node."""

import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch_testing_ros import WaitForTopics
import pytest
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import FluidPressure, Temperature
from std_srvs.srv import Trigger


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch BMP280 node in fake_mode for testing."""
    node = launch_ros.actions.Node(
        package='bmp280_barometer',
        executable='bmp280_node.py',
        name='bmp280_barometer_node',
        parameters=[{
            'fake_mode': True,
            'publish_rate': 50.0,
        }],
    )
    return launch.LaunchDescription([
        node,
        launch_testing.actions.ReadyToTest(),
    ]), {'sensor_node': node}


class TestBMP280Topics(unittest.TestCase):
    """Verify pressure and temperature topics publish valid data."""

    def test_all_topics_published(self):
        """Both pressure and temperature topics should receive messages."""
        topic_list = [
            ('bmp280/pressure', FluidPressure),
            ('bmp280/temperature', Temperature),
        ]
        with WaitForTopics(topic_list, timeout=10.0) as wait:
            self.assertEqual(
                wait.topics_received(),
                {'bmp280/pressure', 'bmp280/temperature'})

    def test_pressure_range(self):
        """Pressure should be near standard atmospheric pressure."""
        topic_list = [('bmp280/pressure', FluidPressure)]
        with WaitForTopics(
            topic_list, timeout=10.0, messages_received_buffer_length=5
        ) as wait:
            msgs = wait.received_messages('bmp280/pressure')
            self.assertGreater(len(msgs), 0)
            for msg in msgs:
                self.assertGreater(msg.fluid_pressure, 90000.0)
                self.assertLess(msg.fluid_pressure, 110000.0)
                self.assertEqual(msg.header.frame_id, 'bmp280_link')

    def test_temperature_range(self):
        """Temperature should be near 25 C (fake_mode default)."""
        topic_list = [('bmp280/temperature', Temperature)]
        with WaitForTopics(
            topic_list, timeout=10.0, messages_received_buffer_length=5
        ) as wait:
            msgs = wait.received_messages('bmp280/temperature')
            self.assertGreater(len(msgs), 0)
            for msg in msgs:
                self.assertGreater(msg.temperature, 15.0)
                self.assertLess(msg.temperature, 35.0)


class TestBMP280Services(unittest.TestCase):
    """Verify calibrate and reset services."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS2 context for service tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shut down ROS2 context."""
        rclpy.shutdown()

    def setUp(self):
        """Create test node."""
        self.node = rclpy.create_node('test_bmp280_services')

    def tearDown(self):
        """Destroy test node."""
        self.node.destroy_node()

    def test_calibrate_service(self):
        """Calibrate should return success in fake mode."""
        client = self.node.create_client(Trigger, 'bmp280/calibrate')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().success)
        self.assertIn('fake', future.result().message.lower())
        self.node.destroy_client(client)

    def test_reset_service(self):
        """Reset should return success."""
        client = self.node.create_client(Trigger, 'bmp280/reset')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().success)
        self.assertIn('reset complete', future.result().message.lower())
        self.node.destroy_client(client)


class TestBMP280Parameters(unittest.TestCase):
    """Verify runtime parameter changes."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS2 context for parameter tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shut down ROS2 context."""
        rclpy.shutdown()

    def setUp(self):
        """Create test node."""
        self.node = rclpy.create_node('test_bmp280_params')

    def tearDown(self):
        """Destroy test node."""
        self.node.destroy_node()

    def test_change_publish_rate(self):
        """Publish_rate should be changeable at runtime."""
        client = self.node.create_client(
            SetParameters, 'bmp280_barometer_node/set_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        request = SetParameters.Request()
        request.parameters = [
            Parameter('publish_rate', value=20.0).to_parameter_msg(),
        ]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().results[0].successful)
        self.node.destroy_client(client)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify clean shutdown."""

    def test_exit_code(self, proc_info):
        """Node should exit cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
