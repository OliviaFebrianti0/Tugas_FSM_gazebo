import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from enum import Enum
import math

class State(Enum):
    FORWARD = 1
    TURN = 2

class FSMNode(Node):
    def __init__(self):
        super().__init__('fsm_node')

        # Publisher ke controller roda
        self.cmd_pub = self.create_publisher(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            10
        )

        # Subscriber LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.state = State.FORWARD
        self.front_distance = float('inf')

        self.timer = self.create_timer(0.1, self.update_fsm)

        self.get_logger().info("FSM + LiDAR STARTED")

    def scan_callback(self, msg):
        # Ambil jarak depan (±10 derajat)
        ranges = msg.ranges
        center = len(ranges) // 2
        window = 10

        front_ranges = ranges[center - window : center + window]
        valid = [r for r in front_ranges if not math.isinf(r)]

        if valid:
            self.front_distance = min(valid)
        else:
            self.front_distance = float('inf')

    def update_fsm(self):
        msg = Twist()

        if self.state == State.FORWARD:
            msg.linear.x = 0.3
            self.cmd_pub.publish(msg)

            if self.front_distance < 0.6:
                self.state = State.TURN
                self.get_logger().info("STATE -> TURN")

        elif self.state == State.TURN:
            msg.angular.z = 0.6
            self.cmd_pub.publish(msg)

            if self.front_distance > 1.0:
                self.state = State.FORWARD
                self.get_logger().info("STATE -> FORWARD")

def main():
    rclpy.init()
    node = FSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
