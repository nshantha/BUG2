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
 

regions = [0,0,0,0,0]
x=0
y=0
theta=0

x1 = 0
y1 = 0
z = 0

count0=0
count1=0

flag = 0
flag_1 = 0

current_position = Point()
goal_position = Point() 

def laser_scan(laser_msg):
    global regions
    regions = [ 
      min(min(laser_msg.ranges[0:71]),8),          #Right
      min(min(laser_msg.ranges[72:143]),8),        #Front Right
      min(min(laser_msg.ranges[144:215]),8),       #Front
      min(min(laser_msg.ranges[216:287]),8),       #Front Left
      min(min(laser_msg.ranges[288:360]),8),       #Left
     ]

def poseCallback(pose_message):  
    global x,y,theta,current_position
    x = pose_message.pose.pose.position.x
    y = pose_message.pose.pose.position.y

    
    rot_q = pose_message.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
    current_position.x = x
    current_position.y = y
    current_position.z = theta

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

def obstacle_avoidance():
    global regions
    reg_values = regions
    vel_msg = Twist()
    print("Heeereee")
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

def flag_shift(f):
    global flag
    flag = f
 


def angle_towards_goal(goal_pos):
    global theta, flag, current_position
    desired_angle = math.atan2(goal_pos.y - current_position.y, goal_pos.x - current_position.x)
    difference_angle = desired_angle - theta
    
    vel_msg = Twist()
    if math.fabs(difference_angle) > 0.05:
        vel_msg.angular.z = 0.7 if difference_angle > 0 else -0.7
    
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1) 
    vel_pub.publish(vel_msg)

    if math.fabs(difference_angle) <= 0.05:
        flag_shift(1)

def distance(position):
    global current_position, goal_position
    i = current_position
    g = goal_position
    num = math.fabs((g.y - i.y) * position.x - (g.x - i.x) * position.y + (g.x * i.y) - (g.y * i.x))
    den = math.sqrt(pow(g.y - i.y, 2) + pow(g.x - i.x, 2))
 
    return num / den if den else 0
    
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

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def done():
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1) 
    vel_pub.publish(vel_msg)

def flag_1_shift(f):
    print("im getting called")
    global flag_1,flag,count1
    count1 = 0
    flag_1 = f
    if flag_1 == 0:
        print("flag_1 is 0")
        if flag == 0:
            print("flag is 0")
            angle_towards_goal(goal_position)
        elif flag == 1:
            print("flag is 1")
            move(goal_position)
        elif flag == 2:
            print("flag is 2")
            done()
        else:
            print("Error")

    if flag_1 == 1:
        print("obstacle")
        obstacle_avoidance()
        
def main():
    
    global goal_position,current_position,regions,count0,count1,flag,flag_1

    rospy.init_node('robot_mover')
    
    rospy.Subscriber("/homing_signal", PoseStamped, goalCallback)
    
    rospy.Subscriber("/base_pose_ground_truth", Odometry, poseCallback)

    laser_sub= rospy.Subscriber("/base_scan", LaserScan, laser_scan)

    flag_1_shift(0)
    # flag_1 = 0
    # flag_shift(0)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        m_line = distance(current_position)
        i=0

        if flag_1 == 0:
            if regions[2] > 0.15 and regions[2] < 1:
                flag_1_shift(1)
 
        elif flag_1 == 1:
            if count1 > 5 and m_line < 0.1:
                flag_1_shift(0)
            

        count0 = count0 + 1
        if count0 == 10:
            count1 = count1 + 1
            count0 = 0
        
        # rate.sleep()

if __name__ == '__main__':
    main()