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

# Region variable for storing laser data
regions = [0,0,0,0,0]

# Variables used to store co-ordinates
x=0
y=0
theta=0

x1 = 0
y1 = 0
z = 0

#Flag Variables
flag = 0
pose_flag = 0

#Position Variables
initial_position = Point()
current_position = Point()
goal_position = Point() 

#Laser Data function
def laser_scan(laser_msg):
    global regions

    regions = [ 
      min(laser_msg.ranges[0:71]),          #Right
      min(laser_msg.ranges[72:143]),        #Front Right
      min(laser_msg.ranges[144:215]),       #Front
      min(laser_msg.ranges[216:287]),       #Front Left
      min(laser_msg.ranges[288:360]),       #Left
     ]

# Callback function for Position of the Robot
def poseCallback(pose_message):  
    global x,y,theta,initial_position,current_position,pose_flag

    if (pose_flag == 0):
        x = pose_message.pose.pose.position.x
        y = pose_message.pose.pose.position.y
    
        rot_q = pose_message.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        initial_position.x = x
        initial_position.y = y
        initial_position.z = theta
        pose_flag = 1

    if (pose_flag == 1):
        x = pose_message.pose.pose.position.x
        y = pose_message.pose.pose.position.y
    
        rot_q = pose_message.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        current_position.x = x
        current_position.y = y
        current_position.z = theta

# Callback function for position of destination
def goalCallback(goal_message):
    # print(goal_message)
    global x1,y1,z,goal_position
    x1 = goal_message.pose.position.x
    y1 = goal_message.pose.position.y

    rot_q = goal_message.pose.orientation
    (roll, pitch, z) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
    goal_position.x = x1
    goal_position.y = y1
    goal_position.z = z

# Function for flag shift
def flag_shift(f):
    global flag
    flag = f

# Function used to direct the robot towards Goal
def angle_towards_goal(goal_pos):
    global theta, flag, current_position
    desired_angle = math.atan2(goal_pos.y - current_position.y, goal_pos.x - current_position.x)
    difference_angle = desired_angle - theta
    
    vel_msg = Twist()
    if math.fabs(difference_angle) > 0.05:
        vel_msg.angular.z = 0.5 if difference_angle > 0 else -0.5
    
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1) 
    vel_pub.publish(vel_msg)

    if math.fabs(difference_angle) <= 0.05:
        flag_shift(1)

# Function used to move Robot towards Goal
def move(goal_pos):
    global theta, flag, current_position
    desired_angle = math.atan2(goal_pos.y - current_position.y, goal_pos.x - current_position.x)
    difference_angle = desired_angle - theta
    difference_pos = math.sqrt(pow(goal_pos.y - current_position.y, 2) + pow(goal_pos.x - current_position.x, 2))
    
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

# Function used for WallFollow
def obstacle_avoidance():
    
    global regions
    reg_values = regions
    vel_msg = Twist()
    
    if reg_values[2] > 1 and reg_values[3] < 1 and reg_values[1] < 1:
        
        vel_msg.linear.x = 0.4
        vel_msg.angular.z = -0.3
    elif reg_values[2] < 1 and reg_values[3] < 1 and reg_values[1] < 1:
        
        
        vel_msg.angular.z = -0.3
    elif reg_values[2] < 1 and reg_values[3] < 1 and reg_values[1] > 1:
        
        vel_msg.linear.x = 0.2
        vel_msg.angular.z = -0.4
    elif reg_values[2] < 1 and reg_values[3] > 1 and reg_values[1] < 1:
        
        vel_msg.linear.x = 0.2
        vel_msg.angular.z = -0.4
    elif reg_values[2] > 1 and reg_values[3] > 1 and reg_values[1] > 1:
        
        vel_msg.linear.x = 0.4
        vel_msg.angular.z = 0.3
    elif reg_values[2] > 1 and reg_values[3] < 1 and reg_values[1] > 1:
        
        vel_msg.linear.x = 0.3
        vel_msg.angular.z = -0.2       
    elif reg_values[2] < 1 and reg_values[3] > 1 and reg_values[1] > 1:
        
        vel_msg.angular.z = -0.3
    elif reg_values[2] > 1 and reg_values[3] > 1 and reg_values[1] < 1:
        
        vel_msg.linear.x = 0.4
    
    
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1) 
    vel_pub.publish(vel_msg)

# Function used to specify destination alert
def reached():
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1) 
    vel_pub.publish(vel_msg)

# function for calculating distance of the 
# position of robot from the m-line
def distance(position):
    global current_position, goal_position
    i = current_position
    g = goal_position
    num = math.fabs((g.y - i.y) * position.x - (g.x - i.x) * position.y + (g.x * i.y) - (g.y * i.x))
    den = math.sqrt(pow(g.y - i.y, 2) + pow(g.x - i.x, 2))
 
    return num / den if den else 0
            
# Main Function
def main():

    global regions, current_position, goal_position
    
    rospy.init_node('bug_2')
    
    # Subscriber for Laser Data
    laser_sub= rospy.Subscriber("/base_scan", LaserScan, laser_scan)

    # Subscriber for Final Position
    rospy.Subscriber("/homing_signal", PoseStamped, goalCallback)
    
    # Subscriber for Position of the robot
    rospy.Subscriber("/base_pose_ground_truth", Odometry, poseCallback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        
        vel_msg =Twist()
        global regions
        reg_values = regions
        dist = distance(initial_position)


        if dist < 0.5 and ((reg_values[2] > 1 and reg_values[3] > 1 and reg_values[1] > 1)):
            if flag == 0:
                angle_towards_goal(goal_position)
            elif flag == 1:
                move(goal_position)
            elif flag == 2:
                reached()
            else:
                print("Error")
        
        elif dist < 0.5 and (reg_values[3] < 1):
            flag_1 = 1
            obstacle_avoidance()           

        elif dist > 0.5:
            obstacle_avoidance()

        elif dist < 0.5 and flag_1 == 1:
            if flag == 0:
                angle_towards_goal(goal_position)
            elif flag == 1:
                move(goal_position)
            elif flag == 2:
                reached()
            else:
                print("Error")

        if goal_position.x == current_position.x and goal_position.y == current_position.y:
            reached()

        rate.sleep()


    rospy.spin()
 
if __name__ == '__main__':
    main() 