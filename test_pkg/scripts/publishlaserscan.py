#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class ObstacleAvoidance:
    def __init__(self):
        self.publisher = rospy.Publisher("/thorvald_001/layo_front_scan", LaserScan, queue_size=10)
        self.subscriber = rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.check_obstacle)
        self.rate = rospy.Rate(10)


    def check_obstacle(self, data):

        # create a new laser scan and copy the incoming laser scan objects into it
        move = LaserScan()
        move.header.frame_id = data.header.frame_id
        move.header.stamp.secs = data.header.stamp.secs
        move.angle_increment = data.angle_increment
        move.time_increment = data.time_increment
        move.range_max = data.range_max
        move.range_min = data.range_min
        move.angle_min = data.angle_min
        move.angle_max = data.angle_max
        
        range = []
        new_range = []
        print('frame: ' + str(data.header.frame_id))
        #convert ranges into cartesian coordinate
        for index, value in enumerate(data.ranges):
            a = data.angle_min + index*data.angle_increment
            x = value*math.cos(a)
            y = value*math.sin(a) 

            (e,f) = (x,y)
            print((x,y))
            
            #filter ranges based on the x coordinate
            #if the x axis fall within the condition below, append the value into angle array for further processing else append 0
            if( x >= 0 and x < 8):
                range.append(value)
            else:
                range.append(0)

        #filter ranges based on the y coordinate
        #if the y axis fall within the condition below, append the value else append 0
        for idx, x in enumerate(range):
            a = data.angle_min + idx*data.angle_increment
            j = x*math.cos(a)
            k = x*math.sin(a) 

            if( k >= -0.5 and k < 12 ):
                new_range.append(x)
            else:
                new_range.append(0)
        

        move.ranges = new_range
        
        self.publisher.publish(move)


if __name__ == '__main__':
    rospy.init_node('publish_laser', anonymous=True)
    make_thorvald_move = ObstacleAvoidance()
    rospy.spin()