import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetEntityState


class RobotTeleopNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        self.robot_descriptions = []
        #read robot description parameters

        self.robot_names = ['tb3_0', 'tb3_1']
        self.robot_reset_poses = []

        self.robot_vel_publishers = []
        #create robot velocity publishers

        #subscibe to keyboard publisher

        #create reset robot client

        self.client_entity_state = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        while not self.client_entity_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for gazebo entity state service')
        
        self.vel_msg = Twist()
        self.reset_world = False
        self.replay_demonstration = False
        self.swapping = False
        self.wait_for_service  = False
        self.robot_id = 0

        #Read the initial pose of the robot
        for robot in self.robot_names:
            robot_state = self.get_model_state(robot)
            self.robot_reset_poses.append(robot_state.state.pose)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        pass

   
    


    def get_model_state(self, model_name):
        """
        Returns state information about a model from gazebo using the models name
        """
        req = GetEntityState.Request()
        req.name = model_name
        self.future = self.client_entity_state.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


    
        

def main(args=None):
    rclpy.init(args=args)

    robot_teleop_node = RobotTeleopNode()

    rclpy.spin(robot_teleop_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()