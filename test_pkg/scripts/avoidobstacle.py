#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoidance:
    def __init__(self):
        self.publisher = rospy.Publisher("/thorvald_001/teleop_joy/cmd_vel", Twist, queue_size=10)
        self.subscriber = rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.check_obstacle)
        self.rate = rospy.Rate(10)


    def check_obstacle(self, data):
        min_laserscan = min(data.ranges)
        move = Twist()

        if min_laserscan < 1.5:
            move.angular.z = 2.5
        else:
            move.linear.x = 0.8
                
        self.publisher.publish(move)


if __name__ == '__main__':
    rospy.init_node('make_thorvald_move', anonymous=True)
    make_thorvald_move = ObstacleAvoidance()
    rospy.spin()