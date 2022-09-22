#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, Point, Twist,PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
import math




class CenterPoint:
    def __init__(self):
        self.path = Path()
        goal = Point()

        self.subscriber = rospy.Subscriber("/point_pub", PointStamped, self.process)
        self.odom = rospy.Subscriber("/thorvald_001/odometry/gazebo", Odometry, self.robot_odometry)
        self.cmd = rospy.Publisher("/thorvald_001/teleop_joy/cmd_vel", Twist, queue_size=10)
        self.path_pub = rospy.Publisher('/patho', Path, queue_size =10)
        self.rate = rospy.Rate(10)


    def robot_odometry(self, msg):
        self.path.header = msg.header
        
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)


    def process(self, data):
        goal = data.point

        angle_to_goal = math.atan2(goal.y, goal.x)

        speed = Twist()

        if (angle_to_goal > -0.05 and goal.y > -0.601):
            print('turn left')
            speed.linear.x = 0.8
            speed.angular.z = 0.1
        elif (angle_to_goal < -0.05  and goal.y < -0.3):
            print('turn right')
            speed.linear.x = 0.8
            speed.angular.z = -0.1
        else :
            print('move')
            speed.linear.x = 0.8
            speed.angular.z = 0.0
        self.cmd.publish(speed)

if __name__ == '__main__':
    rospy.init_node('sub_laser', anonymous=True)
    make_thorvald_move = CenterPoint()
    rospy.spin()