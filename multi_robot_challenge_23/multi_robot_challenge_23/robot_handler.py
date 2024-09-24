import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

class RobotHandlerClass(Node):
    def __init__(self):
        super().__init__('RobotHandlerNode')

        self.sub_lidar = self.create_subscription(LaserScan, 'scan', self.clbk_lidar, 10)
        
        self.pub_namespace_test = self.create_publisher(Float64, 'namespace_test', 10)

        self.lidar_value = 100.0

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def clbk_lidar(self, msg):
        self.lidar_value = msg.ranges[180]

    def timer_callback(self):
        pub_msg = Float64()
        pub_msg.data = self.lidar_value
        self.pub_namespace_test.publish(pub_msg)


def main(args=None):
    rclpy.init(args=args)

    robot_handler = RobotHandlerClass()

    rclpy.spin(robot_handler)

    robot_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()