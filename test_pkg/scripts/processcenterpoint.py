#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math




class CenterPoint:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        goal = Point()


        self.subscriber = rospy.Subscriber("/point_pub", PointStamped, self.process)
        self.odom = rospy.Subscriber("/thorvald_001/odometry/gazebo", Odometry, self.newOdom)
        self.cmd = rospy.Publisher("/thorvald_001/teleop_joy/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)



    def newOdom(self, msg):
        print('received odom')
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        # print('pose: ' + str(rot_q))
        # print()
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        # print('x: ' + str(self.x))
        # print('y: ' + str(self.y))
        # print('z: ' + str(self.theta))

    def process(self, data):
        goal = data.point
        print('x: ' + str(goal.x))
        print('y: ' + str(goal.y))


        inc_x = goal.x - self.x
        inc_y = goal.y - self.y

        angle_to_goal = math.atan2(inc_y, inc_x)
        # print('angle: ' + str(angle_to_goal))
        # print('theta: ' + str(self.theta))
        speed = Twist()
        a = angle_to_goal - self.theta
        print('angle_to_goal' + str(angle_to_goal))
        print('a: ' + str(a))


        if angle_to_goal < -0.05:
            print('turn left')
            speed.linear.x = 0.0
            speed.angular.z = 0.5*a
        elif angle_to_goal > 0.05:
            print('turn right')
            speed.linear.x = 0.0
            speed.angular.z = 0.5*a
        else :
            print('move')
            speed.linear.x = 0.3
            speed.angular.z = 0.0
        self.cmd.publish(speed)

# if a > -0.2 and a < 0.2
if __name__ == '__main__':
    rospy.init_node('sub_laser', anonymous=True)
    make_thorvald_move = CenterPoint()
    rospy.spin()