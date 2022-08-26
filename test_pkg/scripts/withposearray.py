#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
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

class ObstacleAvoidance:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # self.b = TransformBroadcaster()

        self.update = True
        self.updatee = True
        self.twistRobot = True


        self.publisher = rospy.Publisher("/thorvald_001/layo_front_scan", LaserScan, queue_size=10)
        self.cmd = rospy.Publisher("/thorvald_001/teleop_joy/cmd_vel", Twist, queue_size=10)

        self.subscriber = rospy.Subscriber("/thorvald_001/front_scan", LaserScan, self.check_obstacle)
        # self.odom = rospy.Subscriber("/thorvald_001/odometry/gazebo", Odometry, self.newOdom)


        self.cebter_poses = rospy.Publisher("/laser_poses", PoseArray, queue_size=1)
        self.pub_grape_poses2 = rospy.Publisher("/laser_poses2", PoseArray, queue_size=1)
        self.point_pub = rospy.Publisher("/point_pub", PointStamped, queue_size=1)

        self.center_point = rospy.Publisher("/center_point", PointStamped, queue_size=1)
        # self.movebase = rospy.Publisher("/thorvald_001/move_base_simple/goal", PoseStamped, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # self.get_transform()
        self.rate = rospy.Rate(10)
    
    # def get_transform(self):
    #     try:
    #         print('get transform')
    #         self.transform = self.tf_buffer.lookup_transform("thorvald_001/base_link", "thorvald_001/hokuyo_front", rospy.Time(0), rospy.Duration(1.0))
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         print('Error')
    
    # def newOdom(self, msg):
    #     print('received odom')
    #     self.x = msg.pose.pose.position.x
    #     self.y = msg.pose.pose.position.y
    #     (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #     print('x: ' + str(self.x))
    #     print('y: ' + str(self.y))
    #     print('z: ' + str(self.theta))



    def check_obstacle(self, data):
        print('check_obstacle')
        # if(self.update) == True:
        #     goal = Point()
        #     goal.x = 0
        #     goal.y = 0

        #     speed = Twist()
        #     inc_x = goal.x - self.x
        #     inc_y = goal.y - self.y
        #     angle_to_goal = math.atan2(inc_y, inc_x)
        #     print('inc_x: ' + str(inc_x))
        #     print('inc_y: ' + str(inc_y))

        #     print('angle to goal: ' + str(angle_to_goal))
        #     if abs(angle_to_goal - self.theta) > 0.1:
        #         speed.linear.x = 0
        #         speed.angular.z = 0.3
        #     else:
        #         speed.linear.x = 0.5
        #         speed.angular.y = 0
            
        #     if(self.updatee == True):
        #         print('publishing spped')
        #         self.cmd.publish(speed)

        pose_array = PoseArray()
        # pose_array2 = PoseArray()
        pose_array.header.frame_id = "thorvald_001/hokuyo_front"
        # pose_array.header.frame_id = 'layo_front_scan_frame'



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

        right_range = []
        left_range = []
        further_process = []
        left_further_process = []
        right_further_process = []
        kk = 0

        print('batch')
        #convert ranges into cartesian coordinate
        for index, value in enumerate(data.ranges):
            a = data.angle_min + index*data.angle_increment
            x = value*math.cos(a)
            y = value*math.sin(a)

            (e,f) = (x,y)
            
            #filter ranges based on the x coordinateAq1
            #if the x axis fall within the condition below, append the value into angle array for further processing else append 0
            if( x >= 0 and x < 3):
                range.append(value)
            else:
                range.append(0)

            # add point to msg

           

        # filter ranges based on the y coordinate
        # if the y axis fall within the condition below, append the value else append 0
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
        print('new axis')
        # print(new_range)

        if(self.update):
            # self.update = False
            print('update true')
            # print(left_y_axis)
            left_y_axis = [i for i in left_y_axis if i != 0]
            # print('new update true')
            # print(left_y_axis)
            print('left_y_axis')

            left = min(left_y_axis)
            print('min: ' + str(left))

            # print(right_y_axis)
            print('right_y_axis')

            right_y_axis = [i for i in right_y_axis if i != 0]
            right = max(right_y_axis)
            # print(right_y_axis)
            print('max: ' + str(right))

            space = (left + right)/2
            print('space: ' + str(space))

            object_location = Pose()
            object_location.orientation.w = 1.0
            object_location.position.x = 2
            object_location.position.y = space
            pose_array.poses.append(object_location)

            listener = tf.TransformListener()
            listener.waitForTransform("thorvald_001/hokuyo_front", "thorvald_001/base_link", rospy.Time(), rospy.Duration(4.0))

            point = Point()
            point.x = 3
            point.y = space

            point_transformed = PointStamped()
            point_transformed.header.frame_id = "thorvald_001/hokuyo_front"
            point_transformed.header.stamp = rospy.Time(0)
            point_transformed.point = point

            try:
                p = listener.transformPoint("thorvald_001/base_link", point_transformed)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print('Error')

            ang = math.atan2(3,space)
            print('angle: ' + str(ang))
            # speed = Twist()
            # speed.angular.z = ang
            # if(self.twistRobot):
            #     self.twistRobot = False
                # print('in twist')
                # speed = Twist()
                # speed.angular.z = ang
                # self.cmd.publish(speed)
                


            # goal = Point()
            # goal.x = 0
            # goal.y = space
            
            # point = Point()
            # point.x = 3
            # point.y = space
            # point.z = 0

            # stamp = PoseStamped()
            # stamp.header = data.header
            # stamp.pose.position = point

            # pose_transformed = tf2_geometry_msgs.do_transform_pose(stamp, self.transform)

            # point_transformed = PointStamped()
            # point_transformed.header = pose_transformed.header
            # point_transformed.point = pose_transformed.pose.position

            # quart = Quaternion()
            # quart.x = 0
            # quart.y = 0
            # quart.z = 0
            # quart.w = 1

            # poseStamp = PoseStamped()
            # poseStamp.header = data.header
            # poseStamp.pose.position = point
            # poseStamp.pose.orientation = quart
            # translation = (3.0, space, 0.0)
            # rotation = (0.0, 0.0, 0.0, 1.0)

            # # layo_front_scan_frame
            # self.b.sendTransform(translation, rotation, Time.now(), "thorvald_001/hokuyo_front", "thorvald_001/base_link")

            # speed = Twist()
            # inc_x = goal.x - self.x
            # inc_y = goal.y - self.y
            # angle_to_goal = math.atan2(inc_y, inc_x)
            # print('inc_x: ' + str(inc_x))
            # print('inc_y: ' + str(inc_y))

            # print('angle to goal: ' + str(angle_to_goal))
            # if abs(angle_to_goal - self.theta) > 0.1:
            #     speed.linear.x = 0
            #     speed.angular.z = 0.3
            # else:
            #     speed.linear.x = 0.5
            #     speed.angular.y = 0
            
            # if(self.updatee == True):
            #     print('publishing spped')
            #     self.cmd.publish(speed)








            # right_max_x = max(right_x_axis)
            # right_max_y = max(right_y_axis)

            # left_max_y = max(left_y_axis)

            # left_max_x = max(left_x_axis)
            # left_min_x = min(left_x_axis)
            
            # left_min_y = min(left_y_axis)

            # print('x max: ' + str(right_max_x))
            # print('y max: ' + str(right_max_y))

            # print('x max: ' + str(left_max_x))
            # print('y max: ' + str(left_min_x))
            # print('y max: ' + str(left_max_x))
            # print('y max: ' + str(left_min_y))





        # for idx,x in enumerate(axis):
        #     print(x)

            # print('max of j: ' + max(axis[0))  
            # print('min of j: ' + min(j))            
            # print('max of k: ' + max(k))            
            # print('min of k: ' + min(k))            

        move.ranges = new_range
        # move.ranges = left_range


        # print('further_process')
        # print(left_further_process)
        # print('lala')
        # print(further_process[2][0])
        # print(further_process[2][1])
        # print(further_process[2][2])
        self.publisher.publish(move)
        # self.cmd.publish(speed)

        self.cebter_poses.publish(pose_array)
        self.point_pub.publish(p)

        # self.center_point.publish(point_transformed)
        # self.movebase.publish(poseStamp)
        # self.pub_grape_poses2.publish(pose_array)
        # print('transform: ' + str(self.transform))

        #########################################################################################################################
        # Here you've already calculated the y axis above so its a bit odd that you do the calculation again.
        # just use the same one as above with more conditions.
        # so filtering the y is filtering left to right of the robot. you currently filter from 0.5 m to the left of the robot to 12 m to the right of it.
        # you should probably even this filtering.

if __name__ == '__main__':
    rospy.init_node('publish_laser', anonymous=True)
    make_thorvald_move = ObstacleAvoidance()
    rospy.spin()