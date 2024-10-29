#! /usr/bin/env python

import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
# import rospy
# import actionlib
from geometry_msgs.msg import Point
# from bug_2_exercise.msg import bug2_navAction, bug2_navGoal, bug2_navResult, bug2_navFeedback
# from dat160_interfaces.action import Bug2
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ArmTrajectoryController(Node):
    def __init__(self):
        #TODO: Initialze a ros node
        # rospy.init_node('RobotClass')
        super().__init__('ArmTrajectoryController')

        #TODO: Create an action client connected to the "bug2_nav_action" action server.
        # self.bug2_action_client = actionlib.SimpleActionClient("bug2_nav_action", bug2_navAction)
        self.joint_trajectory_action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        #TODO: Wait for the action server to be active
        self.joint_trajectory_action_client.wait_for_server()



    # def done_cb(self, state, result):
    #     rospy.loginfo("The robot finished at position: "+str(result.base_position))

    # def active_cb(self):
    #     rospy.loginfo("Bug2 Navigation has started")

    def feedback_cb(self, feedback_msg):
        desired_pos = feedback_msg.desired.positions
        desired_vel = feedback_msg.desired.velocities

        self.get_logger().info("Desired Position: ", str(desired_pos))
        self.get_logger().info("Desired Velocity: ", str(desired_vel))
        # rospy.loginfo("Current position of the robot: "+str(feedback.current_position))
        # self.get_logger().info("Current position of the robot: "+str(feedback.current_position))

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.joint_trajectory_get_result_future = goal_handle.get_result_async()
        self.joint_trajectory_get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # self.get_logger().info('Result: {0}'.format(result.sequence))
        self.get_logger().info("The robot finished")
        rclpy.shutdown()

    def run(self):
        #TODO: create an action goal with target_position being a Point with x=0 and y=8
        # self.target_position = Point()
        # self.target_position.x = 0.0
        # self.target_position.y = 8.0

        # creating a point
        goal_positions_1 = [math.pi,math.pi/3, math.pi/3, math.pi, math.pi]
        goal_positions_2 = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        point_msg_1 = JointTrajectoryPoint()
        point_msg_1.positions = goal_positions_1
        point_msg_1.time_from_start = Duration(sec=2)

        point_msg_2 = JointTrajectoryPoint()
        point_msg_2.positions = goal_positions_2
        point_msg_2.time_from_start = Duration(sec=4)


        # adding newly created point into trajectory message
        joints = ['arm_base_joint', 'link_1_joint', 'link_2_joint', 'link_3_joint', 'gripper_base_joint']

        # my_trajectory_msg = JointTrajectory()
        # my_trajectory_msg.joint_names = joints
        # my_trajectory_msg.points.append(point_msg_1)
        # my_trajectory_msg.points.append(point_msg_2)
        
        # self.trajectory_publisher.publish(my_trajectory_msg)

        points = []
        points.append(point_msg_1)
        points.append(point_msg_2)

        # goal = bug2_navGoal()
        # goal.target_position = self.target_position
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.goal_time_tolerance = Duration(sec=1)
        goal_msg.trajectory.joint_names = joints
        goal_msg.trajectory.points = points

        # goal_msg.target_position = self.target_position

        #TODO: send the goal to the action server using the existing self.done_cb, self.active_cb and self.feedback_cb functions
        # self.bug2_action_client.send_goal(goal, done_cb=self.done_cb, active_cb=self.active_cb, feedback_cb=self.feedback_cb)
        self.joint_trajectory_send_goal_future = self.joint_trajectory_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)

        self.joint_trajectory_send_goal_future.add_done_callback(self.goal_response_callback)

        #TODO: wait for the action server to finish
        # self.bug2_action_client.wait_for_result()

        return


def main(args=None):
    rclpy.init(args=args)

    robot = ArmTrajectoryController()

    robot.run()

    rclpy.spin(robot)


if __name__ == '__main__':
    main()
