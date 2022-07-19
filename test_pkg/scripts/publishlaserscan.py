#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoidance:
    def __init__(self):
        self.publisher = rospy.Publisher("/thorvald_001/layo_front_scan", LaserScan, queue_size=10)
        self.subscriber = rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.check_obstacle)
        self.rate = rospy.Rate(10)


    def check_obstacle(self, data):
        min_laserscan = min(data.ranges)
        move = LaserScan()
        move.header.frame_id = data.header.frame_id
        move.header.stamp.secs = data.header.stamp.secs
        move.angle_increment = data.angle_increment
        move.time_increment = data.time_increment
        # move.range_max = data.range_max
        # move.range_min = data.range_min
        move.angle_min = data.angle_min
        move.angle_max = data.angle_max
        move.ranges = data.ranges
        # move.angle_min = -1
        # move.angle_max = 1
        move.range_max = 5.0
        move.range_min = data.range_min
        self.publisher.publish(move)


if __name__ == '__main__':
    rospy.init_node('publish_laser', anonymous=True)
    make_thorvald_move = ObstacleAvoidance()
    rospy.spin()