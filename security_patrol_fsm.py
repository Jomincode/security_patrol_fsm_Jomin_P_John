#Jomin/security_patrol_fsm/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np


class SecurityPatrolFSM(Node):
    def __init__(self):
        super().__init__('security_patrol_fsm')

        # Publishers: Velocity commands and critical event alerts
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.alert_pub = self.create_publisher(String, '/robot_alert', 10)

        # Sensor Subscriptions: Planar LiDAR and 9-Axis IMU
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # State Initialization
        self.state = "PATROLLING"
        self.min_front = float('inf')
        self.min_left = float('inf')
        self.min_right = float('inf')
        self.accel_magnitude = 9.81

        # Periodic Behavioral Execution Loop at 10 Hz
        self.timer = self.create_timer(0.1, self.fsm_loop)
        self.get_logger().info("Security Patrol FSM initialized in state: PATROLLING")

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        # Filter out NaN values, zero readings, and self-reflections below 5cm
        ranges = np.where(np.isnan(ranges) | (ranges <= 0.05), float('inf'), ranges)
        if len(ranges) == 0:
            return

        # Slicing planar ranges into functional directional sectors
        front_sector = np.concatenate((ranges[0:20], ranges[-20:]))
        left_sector = ranges[20:70]
        right_sector = ranges[-70:-20]

        # Extract minimum clearance for deterministic threshold evaluation
        self.min_front = float(np.min(front_sector)) if len(front_sector) > 0 else float('inf')
        self.min_left = float(np.min(left_sector)) if len(left_sector) > 0 else float('inf')
        self.min_right = float(np.min(right_sector)) if len(right_sector) > 0 else float('inf')

    def imu_callback(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        # Compute instantaneous Euclidean acceleration magnitude
        self.accel_magnitude = float(np.sqrt(ax**2 + ay**2 + az**2))

    def fsm_loop(self):
        # 1. Critical Priority: ALERT Check
        if self.accel_magnitude > 12.0 and self.state != "ALERT":
            self.transition_to(
                "ALERT",
                f"Kinetic shock detected: |a| = {self.accel_magnitude:.2f} m/s^2 > 12.0 m/s^2"
            )

        # 2. State Dispatcher
        if self.state == "PATROLLING":
            self.execute_patrolling()
        elif self.state == "OBSTACLE_AVOIDANCE":
            self.execute_obstacle_avoidance()
        elif self.state == "ALERT":
            self.execute_alert()

    def execute_patrolling(self):
        if self.min_front < 0.75:
            self.transition_to(
                "OBSTACLE_AVOIDANCE",
                f"Obstacle in path: front = {self.min_front:.2f} m < 0.75 m"
            )
            return
        self.publish_cmd(0.22, 0.0)

    def execute_obstacle_avoidance(self):
        if self.min_front >= 0.75:
            self.transition_to(
                "PATROLLING",
                f"Path cleared: front = {self.min_front:.2f} m >= 0.75 m"
            )
            return

        # Pivot away from tighter side
        angular_z = 0.60 if self.min_left >= self.min_right else -0.60
        self.publish_cmd(0.05, angular_z)

    def execute_alert(self):
        self.publish_cmd(0.0, 0.0)
        alert_msg = String()
        alert_msg.data = f"CRITICAL ALERT: Kinetic shock detected (|a|={self.accel_magnitude:.2f} m/s^2). System halted."
        self.alert_pub.publish(alert_msg)

    def transition_to(self, new_state, trigger_reason):
        self.get_logger().warn(f"[FSM TRANSITION] {self.state} -> {new_state} | Trigger: {trigger_reason}")
        self.state = new_state

    def publish_cmd(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SecurityPatrolFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
