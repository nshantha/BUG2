#!/usr/bin/env python
import rospy
import math
from math import atan2
import time
from geometry_msgs.msg import Twist, Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
 


x=0
y=0
theta=0

x1 = 0
y1 = 0
z = 0

flag = 0
flag_1 = 0

initial_position = Point()
goal_position = Point() 
 
def poseCallback(pose_message):  
    global x,y,theta,initial_position
    x = pose_message.pose.pose.position.x
    y = pose_message.pose.pose.position.y

    
    rot_q = pose_message.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
    initial_position.x = x
    initial_position.y = y
    initial_position.z = theta

def goalCallback(goal_message):
    print(goal_message)
    global x1,y1,z,goal_position
    x1 = goal_message.pose.position.x
    y1 = goal_message.pose.position.y

    rot_q = goal_message.pose.orientation
    (roll, pitch, z) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
    goal_position.x = x1
    goal_position.y = y1
    goal_position.z = z

def flag_shift(state):
    global flag
    flag = state
 
def angle_towards_goal(goal_pos):
    global theta, flag, initial_position
    desired_angle = math.atan2(goal_pos.y - initial_position.y, goal_pos.x - initial_position.x)
    difference_angle = desired_angle - theta
    
    vel_msg = Twist()
    if math.fabs(difference_angle) > 0.05:
        vel_msg.angular.z = 0.7 if difference_angle > 0 else -0.7
    
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1) 
    vel_pub.publish(vel_msg)

    if math.fabs(difference_angle) <= 0.05:
        flag_shift(1)
 
def move(goal_pos):
    global theta, flag, initial_position
    desired_angle = math.atan2(goal_pos.y - initial_position.y, goal_pos.x - initial_position.x)
    difference_angle = desired_angle - theta
    difference_pos = math.sqrt(pow(goal_pos.y - initial_position.y, 2) + pow(goal_pos.x - initial_position.x, 2))
    
    if difference_pos > 0.2:
        vel_msg = Twist()
        vel_msg.linear.x = 0.6
        vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1) 
        vel_pub.publish(vel_msg)
    else:
        flag_shift(2)
    
    # state change conditions
    if math.fabs(difference_angle) > 0.03:
        flag_shift(0)
 
def done():
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1) 
    vel_pub.publish(vel_msg)
 
def listener():
    
    global goal_position

    rospy.init_node('robot_mover')
    
    rospy.Subscriber("/homing_signal", PoseStamped, goalCallback)
    
    rospy.Subscriber("/base_pose_ground_truth", Odometry, poseCallback)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if flag == 0:
            angle_towards_goal(goal_position)
        elif flag == 1:
            move(goal_position)
        elif flag == 2:
            done()
        else:
            print("Error")
        rate.sleep()
 
if __name__ == '__main__':
    listener()