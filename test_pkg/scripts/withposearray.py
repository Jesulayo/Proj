#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray

#########################################################################################################################
## I was a bit confused here because I thought you were going to add a pose array
## Should have here

#########################################################################################################################
import numpy as np
import math

class ObstacleAvoidance:
    def __init__(self):
        self.update = True

        self.publisher = rospy.Publisher("/thorvald_001/layo_front_scan", LaserScan, queue_size=10)
        self.subscriber = rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.check_obstacle)
        self.pub_grape_poses = rospy.Publisher("/laser_poses", PoseArray, queue_size=1)
        self.pub_grape_poses2 = rospy.Publisher("/laser_ppses2", PoseArray, queue_size=1)

        self.rate = rospy.Rate(10)


    def check_obstacle(self, data):

        # create a new laser scan and copy the incoming laser scan objects into it

        '''
        move = LaserScan()
        move = data
        '''
        pose_array = PoseArray()
        pose_array2 = PoseArray()
        pose_array.header.frame_id = 'layo_front_scan_frame'
        pose_array2.header.frame_id = 'layo_front_scan_frame'


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
        x_axis = []
        y_axis = []

        left_x_axis = []
        left_y_axis = []
        right_x_axis = []
        right_y_axis = []

        ##########################################################################################################################
        ## Above this bit this is all good





        #########################################################################################################################
        print('batch')
        #convert ranges into cartesian coordinate
        for index, value in enumerate(data.ranges):
            a = data.angle_min + index*data.angle_increment -(math.pi/4)
            x = value*math.cos(a)
            y = value*math.sin(a)

            (e,f) = (x,y)
            # print((x,y))




            #filter ranges based on the x coordinate
            #if the x axis fall within the condition below, append the value into angle array for further processing else append 0
            if( x >= 0 and x < 20):
                range.append(value)
            else:
                range.append(0)

            # # add point to msg
            # object_location = Pose()
            # object_location.orientation.w = 1.0
            # object_location.position.x = x
            # object_location.position.y = y
            # object_location.position.z = 0.0
            # if(object_location.position.y < 2):
            #     print(object_location.position.x, object_location.position.y)
            #     pose_array2.poses.append(object_location)
            # else:
            #     print('discard')
            # pose_array.poses.append(object_location)

        #########################################################################################################################
        # So here above you are filtering points depending on how far in front of the thorvald it is.
        # range is between 0.5 - 8 m which is why you don't see the blue marked points

        # you can make a pose array topic and publish these points in relation to the frame id of the laser scanner
        # you could also include the tf transformation between the laser frame id and the odom frame to the points and then keep them on the map permenantly while new scans come in



        #########################################################################################################################

        #filter ranges based on the y coordinate
        #if the y axis fall within the condition below, append the value else append 0
        for idx, x in enumerate(range):
            a = data.angle_min + idx*data.angle_increment -(math.pi/4)
            j = x*math.cos(a)
            k = x*math.sin(a)

            if( k >= -2.5 and k < 3 ):
                new_range.append(x)
                
                if( k >= -2.5 and k < 0 ):
                    print('RIGHT')
                    right_x_axis.append(j)
                    right_y_axis.append(k)
                else:
                    print('LEFT')
                    left_x_axis.append(j)
                    left_y_axis.append(k)

            else:
                new_range.append(0)
        print('new axis')

        if(self.update):
            self.update = False
            print('update true')
            right_max_x = max(right_x_axis)
            right_max_y = max(right_y_axis)

            left_max_x = max(left_x_axis)
            left_min_x = min(left_x_axis)
            left_max_x = max(left_y_axis)
            left_min_y = min(left_y_axis)

            # print('x max: ' + str(right_max_x))
            # print('y max: ' + str(right_max_y))

            print('x max: ' + str(left_max_x))
            print('y max: ' + str(left_min_x))
            print('y max: ' + str(left_max_x))
            print('y max: ' + str(left_min_y))



        # for idx,x in enumerate(axis):
        #     print(x)

            # print('max of j: ' + max(axis[0))  
            # print('min of j: ' + min(j))            
            # print('max of k: ' + max(k))            
            # print('min of k: ' + min(k))            

        move.ranges = new_range

        self.publisher.publish(move)
        # self.pub_grape_poses.publish(pose_array)
        # self.pub_grape_poses2.publish(pose_array)

        #########################################################################################################################
        # Here you've already calculated the y axis above so its a bit odd that you do the calculation again.
        # just use the same one as above with more conditions.
        # so filtering the y is filtering left to right of the robot. you currently filter from 0.5 m to the left of the robot to 12 m to the right of it.
        # you should probably even this filtering.

if __name__ == '__main__':
    rospy.init_node('publish_laser', anonymous=True)
    make_thorvald_move = ObstacleAvoidance()
    rospy.spin()