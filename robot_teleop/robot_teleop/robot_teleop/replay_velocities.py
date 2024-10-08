import rclpy
import time
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, Pose
from robot_teleop_interfaces.action import ReplayVel
from nav_msgs.msg import Odometry

class ReplayVelocitiesActionServer(Node):

    def __init__(self):
        super().__init__('replay_velocities_action_server')

        self.current_poses = [Pose(), Pose()]

        self.create_subscription(Odometry, "/tb3_0/odom", self.clbk_odom, 10)
        self.create_subscription(Odometry, "/tb3_1/odom", self.clbk_odom, 10)

        self.robot_vel_publishers = []
        #create robot velocity publishers
        self.robot_vel_publishers.append(self.create_publisher(Twist, '/tb3_0/cmd_vel', 10))
        self.robot_vel_publishers.append(self.create_publisher(Twist, '/tb3_1/cmd_vel', 10))

        self._action_server = ActionServer(
            self,
            ReplayVel,
            '/replay_velocities',
            execute_callback = self.execute_callback,
            cancel_callback=self.clbk_cancel,
        )

        

    def clbk_odom(self, msg):
        # print(f"child frame id: {msg.child_frame_id[5]}")
        # print(f"Frame ID: {msg.header.frame_id[5]}")
        msg_robot_id = int(msg.child_frame_id[5])
        self.current_poses[msg_robot_id] = msg.pose.pose

    def clbk_cancel(self, goal):
        return CancelResponse.ACCEPT


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        robot_id = goal_handle.request.robot_id
        cmd_vel_list = goal_handle.request.cmd_vel_list

        feedback_msg = ReplayVel.Feedback()

        for cmd_vel in cmd_vel_list:
            self.get_logger().info(f"cancel requested: {goal_handle.is_cancel_requested}")
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                result = ReplayVel.Result()
                result.final_pose = self.current_poses[robot_id]
                return result
            feedback_msg.current_pose = self.current_poses[robot_id]
            goal_handle.publish_feedback(feedback_msg)

            self.robot_vel_publishers[robot_id].publish(cmd_vel)
            time.sleep(0.1)
       


        goal_handle.succeed()

        result = ReplayVel.Result()
        result.final_pose = self.current_poses[robot_id]
        return result


def main(args=None):
    rclpy.init(args=args)

    replay_velocities_action_server = ReplayVelocitiesActionServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(replay_velocities_action_server, executor=executor)

    replay_velocities_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()