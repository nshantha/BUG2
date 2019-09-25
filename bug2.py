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

flag = 0
pose_flag = 0


initial_position = Point()
current_position = Point()
goal_position = Point() 

def laser_scan(laser_msg):
    global regions

    regions = [ 
      min(laser_msg.ranges[0:71]),          #Right
      min(laser_msg.ranges[72:143]),        #Front Right
      min(laser_msg.ranges[144:215]),       #Front
      min(laser_msg.ranges[216:287]),       #Front Left
      min(laser_msg.ranges[288:360]),       #Left
     ]

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

def flag_shift(f):
    global flag
    flag = f

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

def obstacle_avoidance():
    # print ("Chalo Chale Mitwa")
    global regions
    reg_values = regions
    vel_msg = Twist()
    # R = regions[0]
    # FR = regions[1]
    # F = regions[2]
    # FL = regions[3]
    # L = regions[4]
    
    if reg_values[2] > 1 and reg_values[3] < 1 and reg_values[1] < 1:
        vel_msg.linear.x = 0.4
        vel_msg.angular.z = -0.3
    elif reg_values[2] < 1 and reg_values[3] < 1 and reg_values[1] < 1:
        vel_msg.angular.z = 0.3
    elif reg_values[2] < 1 and reg_values[3] < 1 and reg_values[1] > 1:
        vel_msg.angular.z = 0.3
    elif reg_values[2] < 1 and reg_values[3] > 1 and reg_values[1] < 1:
        vel_msg.angular.z = 0.3
    elif reg_values[2] > 1 and reg_values[3] > 1 and reg_values[1] > 1:
        vel_msg.linear.x = 0.4
        vel_msg.angular.z = -0.3
    elif reg_values[2] > 1 and reg_values[3] < 1 and reg_values[1] > 1:
        vel_msg.linear.x = 0.4
        vel_msg.angular.z = -0.3       
    elif reg_values[2] < 1 and reg_values[3] > 1 and reg_values[1] > 1:
        vel_msg.angular.z = 0.3
    elif reg_values[2] > 1 and reg_values[3] > 1 and reg_values[1] < 1:
        vel_msg.linear.x = 0.5
    
    
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1) 
    vel_pub.publish(vel_msg)

def reached():
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1) 
    vel_pub.publish(vel_msg)

def distance(position):
    global current_position, goal_position
    i = current_position
    g = goal_position
    num = math.fabs((g.y - i.y) * position.x - (g.x - i.x) * position.y + (g.x * i.y) - (g.y * i.x))
    den = math.sqrt(pow(g.y - i.y, 2) + pow(g.x - i.x, 2))
 
    return num / den if den else 0

# def move_right():
#     vel_msg = Twist()

#     vel_msg.angular.z = -0.3

#     vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1) 
#     vel_pub.publish(vel_msg)

# def move_straight():
#     vel_msg = Twist()

#     vel_msg.linear.z = 0.5

#     vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1) 
#     vel_pub.publish(vel_msg)

# def move_front_right():
#     vel_msg =Twist()

#     vel_msg.linear.x = 0.4
#     vel_msg.angular.z = -0.3   

#     vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size = 1) 
#     vel_pub.publish(vel_msg)


    # flag_1 =0
    # if flag_1 == 0:
    #     if flag == 0:
    #         angle_towards_goal(goal_position)
    #     elif flag == 1:
    #         move(goal_position)
    #         flag_1 == 1
    #     elif flag == 2:
    #         reached()
    #     else:
    #         print("Error")
    # if flag_1 == 1:
    #     if (dist > 0.3 or (reg_values[2] < 0.7 and reg_values[3] < 0.7 and reg_values[1] < 0.7)):
    #         obstacle_avoidance()
    #         flag_1 == 0
    #     else:
    #         flag_1 == 0
            

def main():

    global regions, current_position, goal_position
    
    rospy.init_node('bug_2')
    
    laser_sub= rospy.Subscriber("/base_scan", LaserScan, laser_scan)

    rospy.Subscriber("/homing_signal", PoseStamped, goalCallback)
    
    rospy.Subscriber("/base_pose_ground_truth", Odometry, poseCallback)

    rate = rospy.Rate(10)


    while not rospy.is_shutdown():
        # print(initial_position)
        vel_msg =Twist()
        global regions
        reg_values = regions
        dist = distance(initial_position)
        print(dist)
        # print(dist)
        rate.sleep()

        if dist < 0.5 and not(reg_values[2] < 1 and reg_values[3] < 1):
            if flag == 0:
                angle_towards_goal(goal_position)
            elif flag == 1:
                move(goal_position)
            elif flag == 2:
                reached()
            else:
                print("Error")
        elif dist < 0.5 and (reg_values[2] < 1 or reg_values[3] < 1 or reg_values[1] < 1):
            vel_msg 
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


        # obstacle_avoidance()

    rospy.spin()
 
if __name__ == '__main__':
    main() 