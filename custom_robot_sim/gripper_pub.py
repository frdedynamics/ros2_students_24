import rclpy

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class GripperPublisher(Node):

    def __init__(self):
        super().__init__('gripper_controller')
        
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.gripper_publisher = self.create_publisher(Float64MultiArray, "gripper_controller/commands", 10)


    def timer_callback(self):

        gripper_msg = Float64MultiArray()
        gripper_msg.data = [0.05]

        self.gripper_publisher.publish(gripper_msg)


def main(args=None):

    rclpy.init(args=args)
    gripper_object = GripperPublisher()

    rclpy.spin(gripper_object)
    
    gripper_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()