import rclpy
import copy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetEntityState
from robot_teleop_interfaces.msg import Teleop
from robot_teleop_interfaces.srv import ResetRobot
from robot_teleop_interfaces.action import ReplayVel


class RobotTeleopNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')

        self.robot_descriptions = []
        self.declare_parameter('tb3_0_description', '')
        self.robot_descriptions.append(self.get_parameter('tb3_0_description').get_parameter_value().string_value)
        self.declare_parameter('tb3_1_description', '')
        self.robot_descriptions.append(self.get_parameter('tb3_1_description').get_parameter_value().string_value)
        #read robot description parameters

        self.robot_names = ['tb3_0', 'tb3_1']
        self.robot_reset_poses = []

        self.robot_vel_publishers = []
        #create robot velocity publishers
        self.robot_vel_publishers.append(self.create_publisher(Twist, '/tb3_0/cmd_vel', 10))
        self.robot_vel_publishers.append(self.create_publisher(Twist, '/tb3_1/cmd_vel', 10))

        #subscibe to keyboard publisher
        self.create_subscription(Teleop, 'teleop_device', self.clbk_teleop_device,10)

        #create reset robot client
        self.client_reset_robot = self.create_client(ResetRobot, '/reset_robot')
        while not self.client_reset_robot.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.client_entity_state = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        while not self.client_entity_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for gazebo entity state service')

        self.action_client_replay_vel = ActionClient(self, ReplayVel, '/replay_velocities')
        self.action_client_replay_vel.wait_for_server()
        
        self.vel_msg = Twist()
        self.reset_world = False
        self.replay_demonstration = False
        self.wait_for_replay = False
        self.wait_for_cancel = False
        self.swapping = False
        self.wait_for_service  = False
        self.robot_id = 0
        self.cmd_vel_lists = []
        self.cmd_vel_lists.append([])
        self.cmd_vel_lists.append([])
        self.replay_vel_goal_msg = ReplayVel.Goal()

        #Read the initial pose of the robot
        for robot in self.robot_names:
            robot_state = self.get_model_state(robot)
            self.robot_reset_poses.append(robot_state.state.pose)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def clbk_teleop_device(self, msg):
        self.vel_msg = msg.velocity

        if msg.buttons[0] and not self.swapping:
            self.swapping = True
        elif not msg.buttons[0] and self.swapping:
            self.swapping = False
            if self.robot_id == 0: self.robot_id = 1
            else: self.robot_id = 0

        if msg.buttons[1]:
            self.reset_world = True

        if msg.buttons[2]:
            self.replay_demonstration = True

        if msg.buttons[3] and self.wait_for_replay and not self.wait_for_cancel:
            self.wait_for_cancel = True
            self.get_logger().info(f"Sending cancel goal request")
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.clbk_cancel_done)

    def clbk_cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
            self.wait_for_replay = False
            self.wait_for_cancle = False
            self.replay_demonstration = False
        else:
            self.get_logger().info('Goal failed to cancel')

    def clbk_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self.goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.clbk_get_result)

    def clbk_get_result(self, future):
        result = future.result().result
        self.get_logger().info(f'Final Robot Position: {result.final_pose.position}')
        self.replay_demonstration = False
        self.wait_for_replay = False

    def clbk_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f"Current Robot Position: {feedback.current_pose.position}")

    def timer_callback(self):
        if self.reset_world:
            if not self.wait_for_service:
                reset_request = ResetRobot.Request()
                reset_request.robot_name = self.robot_names[self.robot_id]
                reset_request.robot_description = self.robot_descriptions[self.robot_id]
                reset_request.reset_pose = self.robot_reset_poses[self.robot_id]

                self.reset_robot_future = self.client_reset_robot.call_async(reset_request)
                self.wait_for_service = True

            elif self.wait_for_service and self.reset_robot_future.done():
                self.get_logger().info(f"reset service response: {self.reset_robot_future.result().success}")
                self.wait_for_service = False
                self.reset_world = False

                self.replay_vel_goal_msg = ReplayVel.Goal()
                self.replay_vel_goal_msg.robot_id = self.robot_id
                self.replay_vel_goal_msg.cmd_vel_list = copy.deepcopy(self.cmd_vel_lists[self.robot_id])
                self.cmd_vel_lists[self.robot_id].clear()
        elif self.replay_demonstration:
            if not self.wait_for_replay:
                self.wait_for_replay = True
                self.replay_action_future = self.action_client_replay_vel.send_goal_async(self.replay_vel_goal_msg, feedback_callback=self.clbk_feedback)
                self.replay_action_future.add_done_callback(self.clbk_goal_response)
        else:
            if len(self.cmd_vel_lists[self.robot_id]) == 0 and (self.vel_msg.linear.x != 0.0 or self.vel_msg.angular.z != 0):
                self.cmd_vel_lists[self.robot_id].append(self.vel_msg)
                self.get_logger().info("Adding msg!")
            elif len(self.cmd_vel_lists[self.robot_id]) > 0:
                self.cmd_vel_lists[self.robot_id].append(self.vel_msg)
                self.get_logger().info("Adding msg!")
            self.robot_vel_publishers[self.robot_id].publish(self.vel_msg)


   
    


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