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
        min_laserscan = min(data.ranges)
        move = LaserScan()
        move.header.frame_id = data.header.frame_id
        move.header.stamp.secs = data.header.stamp.secs
        move.angle_increment = data.angle_increment
        move.time_increment = data.time_increment
        move.range_max = data.range_max
        move.range_min = data.range_min
        move.angle_min = data.angle_min
        move.angle_max = data.angle_max
        # move.ranges = data.ranges
        
        # move.angle_min = -1
        # move.angle_max = 1
        # move.range_max = 8
        angle = []
        new_angle = []
        coord = []
        new_coord = []

        print('new batch')
        # print('ange count: ' + str(len(data.ranges)))
        # print('angle_min: ' + str(data.angle_min))
        # adeg = data.angle_min * (180/math.pi)
        # print('angle_min deg: ' + str(adeg))
        # print('angle_max: ' + str(data.angle_max))
        # amaxdeg = data.angle_max * (180/math.pi)
        # print('angle_max deg: ' + str(amaxdeg))
        # print('angle_increment: ' + str(data.angle_increment))
        # amaxdeg = data.angle_increment * (180/math.pi)
        # print('angle_increment deg: ' + str(amaxdeg))

        for index, value in enumerate(data.ranges):
            # print(idx, x)
            a = data.angle_min + index*data.angle_increment
            # print('angle for calculation: ' + str(a))
            # print('cos angle: ' + str(math.cos(a)))
            # print('sin angle: ' + str(math.sin(a)))
            x = value*math.cos(a)
            y = value*math.sin(a) 
            # print('j: ' + str(j))
            # print('k: ' + str(k))

            (e,f) = (x,y)
            print((x,y))
            # angle.append(x)
            # print("the value is: " + str(j) + " " + str(k))
            if( y >= 0 and y < 12):
                coord.append((x,y))
                angle.append(value)
            else:
                # print('appending less than 5')
                angle.append(0)


        # for idx, x in enumerate(data.ranges):
        #     a = data.angle_min + idx*data.angle_increment
        #     j = x*math.cos(a)
        #     k = x*math.sin(a) 

        #     if( k >= -15 and k < 12 ):
        #         # print('appending greater than 5')
        #         new_coord.append((j,k))
        #         new_angle.append(x)
        #     else:
        #         # print('appending less than 5')
        #         new_angle.append(0)
        # print('new angle: ' + str(len(new_angle)))
        move.ranges = angle
        # print('printing coord')
        # print(coord)
        # for idx, x in enumerate(angle):
            # print(x, "reading")
        self.publisher.publish(move)


if __name__ == '__main__':
    rospy.init_node('publish_laser', anonymous=True)
    make_thorvald_move = ObstacleAvoidance()
    rospy.spin()