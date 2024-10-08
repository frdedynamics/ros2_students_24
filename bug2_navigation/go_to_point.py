import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
#TODO: import the custom service type that you have created in your bug2_interfaces package



class GoToPointClass(Node):
    def __init__(self):
        super().__init__('GoToPointController')

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.clbk_odom, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        #TODO: Create Service Server Definition

        self.position = Point()
        self.yaw = 0.0
        self.state = 0
        self.desired_position = Point()
        self.yaw_precision = math.pi / 90
        self.dist_precision = 0.05
        self.active = False

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    #TODO: Create Callback Function for the Service Server that sets the self.active variable True or False AND changes the self.desired_position depending on the request message 

    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def change_state(self, state):
        self.state = state
        #rospy.loginfo('State changed to '+str(self.state))
        self.get_logger().info('State changed to '+str(self.state))

    def normalize_angle(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def done(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.vel_pub.publish(twist_msg)
        self.get_logger().info('go-to-point -> finished')

    def fix_yaw(self, des_pos):
        desired_yaw = math.atan2(des_pos.y - self.position.y, des_pos.x - self.position.x)
        err_yaw = self.normalize_angle(desired_yaw - self.yaw)

        twist_msg = Twist()
        if math.fabs(err_yaw) > self.yaw_precision:
            twist_msg.angular.z = 0.3 if err_yaw > 0.0 else -0.3

        self.vel_pub.publish(twist_msg)
        self.get_logger().info('Yaw error: ['+str(err_yaw)+']')

        #state change condition
        if math.fabs(err_yaw) <= self.yaw_precision:
            self.change_state(1)

    def go_straight_ahead(self, des_pos):
        desired_yaw = math.atan2(des_pos.y - self.position.y, des_pos.x - self.position.x)
        err_yaw = self.normalize_angle(desired_yaw - self.yaw)
        err_pos = math.sqrt(pow(des_pos.y - self.position.y, 2) + pow(des_pos.x - self.position.x, 2))

        if err_pos > self.dist_precision:
            twist_msg = Twist()
            twist_msg.linear.x = 0.3
            self.vel_pub.publish(twist_msg)
        else:
            self.get_logger().info('Position error: '+str(err_pos))
            self.change_state(2)

        # state change condition
        if math.fabs(err_yaw) > self.yaw_precision:
            self.change_state(0)

    def timer_callback(self):
        if not self.active:
            dist_to_target = math.sqrt(pow(self.desired_position.y - self.position.y, 2) + pow(self.desired_position.x - self.position.x, 2))
            if dist_to_target < 0.15:
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.vel_pub.publish(twist_msg)
            return
        
        if self.state == 0:
            self.fix_yaw(self.desired_position)
        elif self.state == 1:
            self.go_straight_ahead(self.desired_position)
        elif self.state == 2:
            self.done()
            self.active = False
            
        else:
            self.get_logger().error('Unknown state!')
            

def main(args=None):
    rclpy.init(args=args)

    controller = GoToPointClass()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
