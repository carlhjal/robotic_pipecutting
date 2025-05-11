#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import csv

class JointStateLogger(Node):
    def __init__(self):
        super().__init__('joint_state_logger')

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)
        self.create_subscription(Bool, '/start_logging', self.start_logging_callback, 10)

        # TF Listener for EE pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Frame IDs — adjust to your robot
        self.base_frame = 'ur20_base_link'
        self.ee_frame = 'tool_endpoint'

        # Logging setup
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path(get_package_share_directory("eval_tools")) / "output"
        output_dir.mkdir(parents=True, exist_ok=True)
        self.filepath = output_dir / f"joint_states_estimated_{timestamp}.csv"
        self.csvfile = open(self.filepath, 'w', newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow([
            'time', 'joint', 'position', 'velocity', 'acceleration', 'jerk',
            'ee_x', 'ee_y', 'ee_z', 'ee_qx', 'ee_qy', 'ee_qz', 'ee_qw'
        ])

        self.get_logger().info(f"Waiting for /start_logging trigger...")
        self.get_logger().info(f"Logging to {self.filepath}")

        self.log_enabled = False
        self.last_positions = {}
        self.last_velocities = {}
        self.last_accelerations = {}
        self.last_time = None

    def start_logging_callback(self, msg: Bool):
        if msg.data:
            self.log_enabled = True
            self.get_logger().info("Received /start_logging trigger — logging enabled.")

    def get_ee_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            t = trans.transform.translation
            r = trans.transform.rotation
            return [t.x, t.y, t.z, r.x, r.y, r.z, r.w]
        except Exception:
            return [None] * 7

    def listener_callback(self, msg: JointState):
        if not self.log_enabled:
            return

        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = current_time
            self.last_positions = dict(zip(msg.name, msg.position))
            return

        dt = current_time - self.last_time
        if dt <= 0:
            return

        ee_pose = self.get_ee_pose()

        for i, name in enumerate(msg.name):
            pos = msg.position[i]
            last_pos = self.last_positions.get(name, pos)
            vel = (pos - last_pos) / dt
            last_vel = self.last_velocities.get(name, 0.0)
            acc = (vel - last_vel) / dt
            last_acc = self.last_accelerations.get(name, 0.0)
            jerk = (acc - last_acc) / dt

            self.writer.writerow([
                current_time, name, pos, vel, acc, jerk,
                *ee_pose
            ])

            self.last_positions[name] = pos
            self.last_velocities[name] = vel
            self.last_accelerations[name] = acc

        self.last_time = current_time

    def destroy_node(self):
        self.csvfile.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointStateLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, exiting.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
