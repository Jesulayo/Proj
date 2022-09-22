#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseArray, Point, PointStamped, PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import numpy as np
import math
import tf
import tf2_ros
import tf2_geometry_msgs
from rospy import Time
from tf import TransformBroadcaster

class DetectFreeSpace:
    def __init__(self):

        # The filtered laser scan is piblished on /thorvald_001/filtered_scan topic
        # The center point pose array is published on /laser_poses
        # Th center point in the robot base_link frame is published on /point_pub
        # The can be visualized on RVIZ by listening to them

        self.update = True

        self.listener = tf.TransformListener()
        self.listener.waitForTransform("thorvald_001/hokuyo_front", "thorvald_001/base_link", rospy.Time(), rospy.Duration(4.0))

        self.publisher = rospy.Publisher("/thorvald_001/filtered_scan", LaserScan, queue_size=10)

        self.subscriber = rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.check_obstacle)

        self.center_pose = rospy.Publisher("/laser_poses", PoseArray, queue_size=1)
        self.point_pub = rospy.Publisher("/point_pub", PointStamped, queue_size=1)


        self.center_point = rospy.Publisher("/center_point", PointStamped, queue_size=1)
        self.rate = rospy.Rate(10)
    

    def check_obstacle(self, data):
        pose_array = PoseArray()
        pose_array.header.frame_id = "thorvald_001/hokuyo_front"



        move = LaserScan()
        move.header.frame_id = data.header.frame_id
        move.header.stamp.secs = data.header.stamp.secs
        move.angle_increment = data.angle_increment
        move.time_increment = data.time_increment
        move.range_max = data.range_max
        move.range_min = data.range_min
        move.angle_min = data.angle_min
        move.angle_max = data.angle_max

        # Arrays for storing required laser scans

        range = []
        new_range = []
        x_axis = []
        y_axis = []

        left_x_axis = []
        left_y_axis = []
        right_x_axis = []
        right_y_axis = []

        right_range = []
        left_range = []
        further_process = []
        left_further_process = []
        right_further_process = []

        #convert ranges into cartesian coordinate
        for index, value in enumerate(data.ranges):
            a = data.angle_min + index*data.angle_increment
            x = value*math.cos(a)
            y = value*math.sin(a)

            (e,f) = (x,y)
            
            #filter ranges based on the x coordinate
            #if the x axis fall within the 0 and 3, append the value into array for further processing else append 0
            if( x >= 0 and x < 3):
                range.append(value)
            else:
                range.append(0)


           

        # filter ranges based on the y coordinate
        # if the y axis fall within the -3 and 3, append the value else append 0
        for idx, x in enumerate(range):
            a = data.angle_min + idx*data.angle_increment
            j = x*math.cos(a)
            k = x*math.sin(a)

            if( k >= -3 and k < 3 ):
                new_range.append(x)
                further_process.append((x,j,k))
                
                if( k >= -3 and k < 0 ):
                    # print('RIGHT')
                    right_further_process.append((x,j,k))
                    right_range.append(x)
                    right_x_axis.append(j)
                    right_y_axis.append(k)
                    
                else:
                    right_further_process.append((0,0,0))
                    right_range.append(0)

                if( k >= 0 and k < 3 ):
                    left_further_process.append((x,j,k))
                    left_range.append(x)
                    # print('LEFT')
                    left_x_axis.append(j)
                    left_y_axis.append(k)
                else:
                    left_further_process.append((0,0,0))
                    left_range.append(0)
                    

            else:
                new_range.append(0)
                right_range.append(0)
                left_range.append(0)

        if(self.update):
            # remove zeros fromy the left side array
            left_y_axis = [i for i in left_y_axis if i != 0]

            # determine the minimum value from the left side array
            left = min(left_y_axis)

            # remove zeros fromy the right side array
            right_y_axis = [i for i in right_y_axis if i != 0]

            # determine the maximum value from the right side array
            right = max(right_y_axis)

            # get the coordinate of the center point of the free space
            space = (left + right)/2

            object_location = Pose()
            object_location.orientation.w = 1.0
            object_location.position.x = 2
            object_location.position.y = space
            pose_array.poses.append(object_location)


            point = Point()
            point.x = 2
            point.y = space

            point_transformed = PointStamped()
            point_transformed.header.frame_id = "thorvald_001/hokuyo_front"
            point_transformed.header.stamp = rospy.Time(0)
            point_transformed.point = point

            # transform the center point from the lidar frame to the robot's base_link frame

            try:
                self.p = self.listener.transformPoint("thorvald_001/base_link", point_transformed)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print('Error') 

        

        move.ranges = new_range
        
        self.publisher.publish(move)

        self.center_pose.publish(pose_array)
        self.point_pub.publish(self.p)

if __name__ == '__main__':
    rospy.init_node('publish_laser', anonymous=True)
    make_thorvald_move = DetectFreeSpace()
    rospy.spin()