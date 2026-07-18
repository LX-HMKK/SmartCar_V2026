import math
import os
import signal
import subprocess
import threading
import time
import unittest


class EkfSetPoseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.domain_id = 225
        os.environ["ROS_DOMAIN_ID"] = str(cls.domain_id)
        os.environ["ROS_LOCALHOST_ONLY"] = "1"

        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from robot_localization.srv import SetPose
        from sensor_msgs.msg import Imu

        cls.rclpy = rclpy
        cls.Odometry = Odometry
        cls.Imu = Imu
        cls.SetPose = SetPose
        cls.latest_output = None
        cls.output_condition = threading.Condition()

        launch_environment = os.environ.copy()
        cls.launch_process = subprocess.Popen(
            ["ros2", "launch", "origincar_base", "ekf.launch.py"],
            env=launch_environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )

        rclpy.init()
        cls.node = rclpy.create_node("ekf_set_pose_test")
        cls.odom_publisher = cls.node.create_publisher(Odometry, "/odom", 10)
        cls.imu_publisher = cls.node.create_publisher(Imu, "/imu/data_raw", 10)
        cls.node.create_subscription(
            Odometry, "/odom_combined", cls._on_filtered_odom, 10)
        cls.set_pose_client = cls.node.create_client(SetPose, "/set_pose")
        cls.timer = cls.node.create_timer(0.05, cls._publish_zero_sensors)

        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.executor_thread = threading.Thread(
            target=cls.executor.spin,
            name="ekf-set-pose-executor",
            daemon=True,
        )
        cls.executor_thread.start()

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls, "timer"):
            cls.timer.cancel()
        if hasattr(cls, "executor"):
            cls.executor.shutdown(timeout_sec=2.0)
        if hasattr(cls, "executor_thread"):
            cls.executor_thread.join(timeout=2.0)
        if hasattr(cls, "node"):
            cls.node.destroy_node()
        if hasattr(cls, "rclpy") and cls.rclpy.ok():
            cls.rclpy.shutdown()

        process = getattr(cls, "launch_process", None)
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3.0)

    @classmethod
    def _on_filtered_odom(cls, message):
        with cls.output_condition:
            cls.latest_output = message
            cls.output_condition.notify_all()

    @staticmethod
    def _set_diagonal(covariance, diagonal):
        dimension = int(math.sqrt(len(covariance)))
        for index, value in enumerate(diagonal):
            covariance[index * dimension + index] = value

    @classmethod
    def _publish_zero_sensors(cls):
        stamp = cls.node.get_clock().now().to_msg()

        odom = cls.Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.w = 1.0
        cls._set_diagonal(
            odom.pose.covariance,
            [0.25, 0.25, 1e6, 1e6, 1e6, 0.50],
        )
        cls._set_diagonal(
            odom.twist.covariance,
            [0.04, 0.01, 1e6, 1e6, 1e6, 0.25],
        )
        cls.odom_publisher.publish(odom)

        imu = cls.Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "base_footprint"
        imu.orientation.w = 1.0
        imu.orientation_covariance[0] = -1.0
        cls._set_diagonal(
            imu.angular_velocity_covariance,
            [1.0, 1.0, 0.02],
        )
        cls._set_diagonal(
            imu.linear_acceleration_covariance,
            [0.25, 0.25, 0.25],
        )
        cls.imu_publisher.publish(imu)

    @classmethod
    def _wait_for_output(cls, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        with cls.output_condition:
            while time.monotonic() < deadline:
                message = cls.latest_output
                if message is not None and predicate(message):
                    return message
                cls.output_condition.wait(
                    timeout=max(0.0, deadline - time.monotonic()))
        return None

    @staticmethod
    def _finite_output(message):
        values = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.angular.z,
        )
        return all(math.isfinite(value) for value in values)

    def test_ekf_output_and_set_pose_without_hardware_nodes(self):
        self.assertTrue(self.set_pose_client.wait_for_service(timeout_sec=15.0))
        initial = self._wait_for_output(self._finite_output, 15.0)
        self.assertIsNotNone(initial, self._launch_output())
        self.assertEqual(initial.header.frame_id, "odom_combined")
        self.assertEqual(initial.child_frame_id, "base_footprint")

        node_names = set(self.node.get_node_names())
        self.assertIn("ekf_filter_node", node_names)
        self.assertNotIn("origincar_base", node_names)
        self.assertEqual(self.node.get_publishers_info_by_topic("/cmd_vel"), [])

        request = self.SetPose.Request()
        request.pose.header.stamp = self.node.get_clock().now().to_msg()
        request.pose.header.frame_id = "odom_combined"
        request.pose.pose.pose.position.x = 1.25
        request.pose.pose.pose.position.y = -0.5
        request.pose.pose.pose.orientation.w = 1.0
        self._set_diagonal(
            request.pose.pose.covariance,
            [0.25, 0.25, 1e6, 1e6, 1e6, 0.50],
        )

        future = self.set_pose_client.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(future.done(), self._launch_output())
        self.assertIsNotNone(future.result())

        def reset_pose_received(message):
            return (
                self._finite_output(message)
                and abs(message.pose.pose.position.x - 1.25) <= 0.05
                and abs(message.pose.pose.position.y + 0.5) <= 0.05
            )

        reset_output = self._wait_for_output(reset_pose_received, 5.0)
        self.assertIsNotNone(reset_output, self._launch_output())

    @classmethod
    def _launch_output(cls):
        process = cls.launch_process
        if process.poll() is None or process.stdout is None:
            return "EKF launch is still running without a matching output"
        return process.stdout.read()


if __name__ == "__main__":
    unittest.main()
