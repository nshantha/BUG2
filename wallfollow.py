#!/usr/bin/env python
import rospy
import math
from math import atan2
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

regions = [0,0,0,0,0]

x=0
y=0
theta=0

x1 = 0
y1 = 0
z = 0

# def poseCallback(pose_message):  
#     global x,y,theta
#     x = pose_message.pose.pose.position.x
#     y = pose_message.pose.pose.position.y

    
#     rot_q = pose_message.pose.pose.orientation
#     (roll, pitch, theta) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])

# def goalCallback(goal_message):
#     print(goal_message)
#     global x1,y1,z
#     x1 = goal_message.pose.position.x
#     y1 = goal_message.pose.position.y

#     rot_q = goal_message.pose.orientation
#     (roll, pitch, z) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])

def obstacle_avoidance():
    global regions
    reg_values = regions
    vel_msg = Twist()

    if reg_values[2] > 0.7 and reg_values[3] < 0.7 and reg_values[1] < 0.7:
        vel_msg.linear.x = 0.4
        vel_msg.angular.z = -0.3
    elif reg_values[2] < 0.7 and reg_values[3] < 0.7 and reg_values[1] < 0.7:
        vel_msg.angular.z = 0.3
    elif reg_values[2] < 0.7 and reg_values[3] < 0.7 and reg_values[1] > 0.7:
        vel_msg.angular.z = 0.3
    elif reg_values[2] < 0.7 and reg_values[3] > 0.7 and reg_values[1] < 0.7:
        vel_msg.angular.z = 0.3
    elif reg_values[2] > 0.7 and reg_values[3] > 0.7 and reg_values[1] > 0.7:
        vel_msg.linear.x = 0.4
        vel_msg.angular.z = -0.3
    elif reg_values[2] > 0.7 and reg_values[3] < 0.7 and reg_values[1] > 0.7:
        vel_msg.linear.x = 0.4
        vel_msg.angular.z = -0.3       
    elif reg_values[2] < 0.7 and reg_values[3] > 0.7 and reg_values[1] > 0.7:
        vel_msg.angular.z = 0.3
    elif reg_values[2] > 0.7 and reg_values[3] > 0.7 and reg_values[1] < 0.7:
        vel_msg.linear.x = 0.5
    else:
        print("No condition")
    
    
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1) 
    vel_pub.publish(vel_msg)


def laser_scan(laser_msg):
    global regions
    regions = [ 
      min(laser_msg.ranges[0:71],8),          #Right
      min(laser_msg.ranges[72:143],8),        #Front Right
      min(laser_msg.ranges[144:215],8),       #Front
      min(laser_msg.ranges[216:287],8),       #Front Left
      min(laser_msg.ranges[288:360],8),       #Left
     ]

    obstacle_avoidance()
 
def main():
    rospy.init_node('robot_mover')
    
    laser_sub= rospy.Subscriber("/base_scan", LaserScan, laser_scan)

    # rospy.Subscriber("/homing_signal", PoseStamped, goalCallback)
    
    # rospy.Subscriber("/base_pose_ground_truth", Odometry, poseCallback)
 
    rospy.spin()
 
if __name__ == '__main__':
    main()