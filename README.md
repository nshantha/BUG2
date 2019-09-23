# ROS_BUG2
CSE 468/568: Robotic Algorithms
Laser-Based Perception and
Navigation using Obstacle Avoidance
VERSION 1.1
PA1, Fall 2019
DEADLINE: September 25, 2019 11:59 pm
INSTRUCTOR: Vivek Thangavelu
WRITTEN BY: Vivek Thangavelu and Zijan An
I. Objective
A 2 wheeled robot is equipped with a 2D laser range finder in order to detect obstacles in its
environment. It needs to get to its charging station using a homing signal. The objective of this
assignment is to perform perception using a laser range finder, and use the perceived information
to avoid obstacles and navigate to the charging station.
II. Description
1. ROS Package
You are supplied with a ROS Package named ros_pa1 .
● homing_beacon.py - A ROS node written in python that emits the homing signal as a ROS
topic /homing_signal.
● sim.launch - Brings up the simulator node from the installed package stage_ros and the
homing_beacon node.
● sim_rviz.launch - Brings up the simulator node stage_ros, rviz and the homing_beacon
node.
2. Deliverables
1. You will write node(s) to read the sensor data, make sense of the sensor information and
implement the BUG2 algorithm.
2. Wrap your node(s) using a launch file named pa1.launch and place it inside the launch
folder of the package ros_pa1. Running the above launch file should bring up the
simulator, rviz and all the nodes required to implement the BUG2 algorithm.
III. Setup
1. Download the ROS base package for PA1
(SHA256: e2362d26b5ef31651cf0591f817dcbbdaf9ff199e944ba553937c98532c01c65)
2. Extract the contents of the tarball into ~/catkin_ws/src . After extraction, you should have a
folder named ros_pa1 inside ~/catkin_ws/src
3. Grant executable permissions to homing_beacon.py
chmod +x ~/catkin_ws/src/ros_pa1/scripts/homing_beacon.py
4. Run catkin_make and source your bashrc file to inform the ROS filesystem about your
new package :
cd ~/catkin_ws
catkin_make
source ~/.bashrc
5. Run one of the launch files. Ex:
roslaunch ros_pa1 sim_rviz.launch
IV. Concepts:
1. Simulator
A Simulator usually runs as a run-time emulator of physical hardware. A robotics simulator is
used to create an application for a physical robot without depending on the actual machine, thus
saving cost and time. In some cases, these applications can be transferred onto the physical robot
(or rebuilt) without modifications.
Stage is a 2(.5)D robotics standalone simulator. It provides a virtual world populated by mobile
robots and sensors, along with various objects for the robots to sense and manipulate. The
stageros node wraps the Stage 2-D multi-robot simulator, via libstage for ROS. Stage simulates a
world as defined in the playground.world file, present inside the world sub-folder. This file tells
stage everything about the world, from obstacles (usually represented via a bitmap to be used as
a kind of background), to robots and other objects.
You can change the view in the simulator and move the robot in stageros. You can add some basic
visualizations such as the laser scanner range data or the robot footprints for debugging from the
View menu in stageros.
2. Visualization
A visualization tool helps to display sensor data and a robot’s state information. rviz (ROS
visualization) is a 3D visualizer from ROS. It provides a convenient GUI to visualize sensor data,
robot models and environment maps, which is useful for developing and debugging your robot
controllers.
In rviz, you can visualize the laser scan information. The robot frame of reference is named
“base_link” and the frame shows the pose of the robot in the “odom” frame. “odom” frame is the
absolute world frame. For more information on frames, refer Section IV.8.
NOTE: Most of the visual information for PA1 should be available in the simulator and you can use
rviz to help you in debugging/visualizing the sensor data. However, it is not a requirement for you
to make use of any other rviz functionality for this programming assignment.
3. Robot Motion
The robot in stageros simulates a 2 wheeled differential drive robot that drives over a smooth
ground plane. In order to move the robot in the simulator, we need to send velocity commands to
the robot. This is done by publishing command geometry_msgs/Twist messages to the topic
/cmd_vel . The robot can only respond to linear velocities in the x axis and angular velocities in the
z axis. So, to move forward, publish Twist messages with non-zero linear.x velocities and to turn,
publish Twist messages with non-zero angular.z values .
A simple behaviour to move your robot without using the more complicated differential model is
to always have only an angular or linear velocity at a specific time to turn or move forward,
respectively.
To know the pose of the robot, you can subscribe to /base_pose_ground_truth of type
nav_msgs/Odometry and retrieve the position and orientation from the pose member variable.
Orientations are expressed as quaternions in ROS. Please refer to section VII.11 for conversion
between quaternion and euler angles.
4. Sensor
In order to make sense of its environment, the robot needs to get data from its sensors. The robot
is equipped with a planar laser range-finder that gives information about obstacles around the
robot. The sensor has 361 lines of lasers allowing the robot to perceive 180 degrees of the world
around it at a maximum range of 3m. In your stageros simulator, you can view the robot sensing
range by clicking on View -> Data or pressing the key ‘ D ’.
The sensor data is published to a topic /base_scan which is of type sensor_msgs/LaserScan . Use
the ranges[i] member variable to get the range information of a particular laser line i. If there is
an obstacle between range_min and range_max, range will be the distance of the sensed
obstacle from the sensor. If an obstacle is outside its range values, the range will be equal to
range_max . Move the robot around by clicking and dragging it in the simulator and visualize the
output of the sensor in rviz to understand how the sensor works. Subscribe to the topic and read
the messages to get the values of range_max, range_min , each range data, etc.
5. Homing Signal
The robot constantly receives a homing signal that tells the robot where the charging station is.
The topic /homing_signal of type geometry_msgs/PoseStamped can be used to read the location
of the charging station.
6. Robot State
Your robot is primarily in one of two states/behaviors: GOALSEEK or WALLFOLLOW . You will need
to use the sensor information to estimate which state the robot must be in and then define what
the robot must do in each behaviour.
The laser range finder provides us with a very high resolution sensing information. We can
simplify the laser range finder data by splitting the sensor’s viewing area into equally divided
circular sectors. In Fig. 1, we have split the laser viewing area into 5 equal sectors: FRONT, FRONT
LEFT, LEFT, FRONT RIGHT, RIGHT. We can now estimate the closest obstacle in each sector.
Knowing the presence and distance of obstacles in each of these sectors, we can make sense of
the nature of obstacles in the view. For example, we can make use of obstacle information only in
the FRONT, FRONT LEFT and FRONT RIGHT sector and deduce eight possible cases:
Fig. 1: The robot (in green) is depicted with a 180 degree view of laser scans (in blue) sectioned into 5 groups
(L - Left, FL - Front Left, F - Front, FR - Front Right, R - Right)
FRONT LEFT FRONT FRONT RIGHT
X
X
X
X X
X X
X X
X X X
Table 1: 8 Possible cases with a usable region of Front Left, Front and Front Right.
X represent the presence of an obstacle in a sector
Using the above possible cases for the three regions, we can figure out the nature of the obstacle
in front of the robot and estimate which state the robot must be in. For example, if an obstacle is
present only in the front sector, we can implement a sub-behaviour to move further towards the
obstacle until we sense an obstacle in all the three sectors (F, FL, and FR) and eventually switch to
a WALLFOLLOW state. You may also deem some cases to be invalid behaviours, tweak the
number of sectors, implement interim behaviour(s), etc. in your implementation.
7. Goal
You should be able to estimate whether your robot has reached its goal and subsequently stop
the robot. You can add a small threshold around the actual goal position and if your robot is
within this small threshold region, then you can flag it as reached and stop the robot.
8. Coordinate Frames
In this robot system, there are 4 coordinate frames of reference. For this PA, we only care about
the following aspects of the various frames of reference:
● The absolute coordinate frame of reference is named odom i.e. the root of the tree data
structure that represents the various frames.
● The robot’s frame of reference is the base_link frame
● The robot pose is expressed in the odom frame
● The goal position is expressed in the odom frame
● Laser scan data is expressed in the base_laser_link frame. The laser scan is mounted such
that its center of mass coincides with the center of mass of the robot and hence the range
distances from the laser sensor in the base_laser_link frame is essentially measured from
the robot’s center of mass.
You do not have to worry about the other frames that exist in the system as all the pose
information is expressed in the same frame ( odom) and the distances of the obstacles are
measured from the robot’s frame. You still need to perform some basic algebraic operations to
find the goal position in the robot’s frame so that you may estimate the robot velocities. If you are
curious about how frames work in ROS, please refer to Section VI.10.
V. Implementation Tips
Implementation in robotics can be a daunting task with multiple sub-objectives in robot control,
sensor reading , feedback control using sensor data, and the obstacle avoidance algorithm. It is
always a good idea to list out the sub-objectives for your task and modularize your code
accordingly. Below is a set of sub-objectives that might help you organize your work for this PA.
You do not have to follow this strictly and is presented here purely as an example.
1. Get the robot to move;
2. Read the sensor data;
3. Simple motion control for a random obstacle avoidance: If too close to any object in view,
move away from it. This step will give a good idea of your understanding of sensor-motion
feedback;
4. Get goal information;
5. Divide sensor data into sectors and deduce the possible cases;
6. Implement the two robot states/behaviour (and other interim behaviours that may help
you switch between them):
a. GOALSEEK: Use goal information to come up with robot velocity commands
b. WALLFOLLOW: Use sensor information to detect walls and implement a behavior to
follow it
7. Write an algorithm module to wrap around the various robot behaviours and implement
the BUG2 algorithm.
VI. Submission Instructions
We will be using autolab in this course to submit your programming assignments. In the autolab
course page, select pa1 and then submit a tarball of your entire ros_pa1 folder. From the file
viewer, you can right click ros_pa1 and click compress to get a tarball of the entire folder. The
deadline for submission is September 25, 2019 11:59 pm . The deadline will be strictly enforced so
please submit once much earlier to test out the system. You are allowed to make multiple
submissions. We will use the final submission version to grade your programming assignment.
Late Submission Policy
You may choose to submit the assignment late by a maximum of 2 days. Each day you lose 25% of
the full grade. Hence, by submitting one day late, you lose 25% and by day 2, you lose 50%. The
deadline for each extra day is 11:59 pm .
VII. FAQ
1. What will I learn from this PA?
At the end of the PA1, you will have learned to:
a. Harness the power of ROS
b. Control a simulated robot
c. Understand laser perception
d. Implement autonomous robot behaviour using an obstacle avoidance algorithm.
2. How to find the type of a topic?
Use the rostopic command line tool.
3. How to import/include a message module?
Every message is defined inside a package. So you must first know the name of the package and
the message type. For example, /cmd_vel is of type geometry_msgs/Twist. geometry_msgs is the
name of the package and Twist is the type of message.
To import in Python :
from <pkg_name>.msg import <msg_type>
To include in C++:
#include “<pkg_name>/<msg_type>.h”
4. I keep crashing into walls (not figuratively)
Make sure you are using the sensor data as planned. You robot might execute a certain motion
behaviour when an obstacle is sensed at a specific range. You might need to tweak this based on
how fast your robot is moving or change the distance at which this behaviour is executed. For
example, if this range is too low, your robot may often bump into walls before it can align itself to
one.
5. rosrun does not run my node
Python:
a. Make sure your python script is in the scripts folder of your ROS package
b. Make sure you have given executable privileges to your python script:
chmod +x <script_name>.py
c. Add the python header to your script:
#!/usr/bin/env python
C++:
a. Make the necessary changes to the CMakeLists.txt file inside your package folder
b. Make sure you ran catkin_make and sourced your bashrc file:
catkin_make
source ~/.bashrc
6. Can I port my code to a physical robot?
YESSSS! With a few tweaks in your robot parameters we can easily run your obstacle avoidance
program on a real live physical robot. Having a modular code can help :).
7. I cannot get the homing signal from the topic /homing_signal.
a. Make sure the node homing_beacon is running:
rosnode list
b. Make sure you are subscribing to the right topic
c. Make sure you have given executable privileges to homing_beacon .py (Refer VII.5b)
8. Can I change the goal position or initial position?
Yes, you can change the initial position of the robot in the playground.world file within the world
folder of your ros_pa1 package. The goal position is published by homing_bacon.py, present
within the scripts folder. We will test the robustness of your implementation against many initial
and goal positions.
9. I need the homing_signal only once, but how do I fit this into a continuous
publisher/subscriber model?
Good question. Since the homing_signal is in the absolute frame(odom), it always has the same
position no matter where the robot is in the world. You can come up with some hacks/clever ways
to read the /homing_signal topic just once, or you may use rospy.wait_for_message in Python or
ros::waitForMessage in C++.
10. How are frames expressed in ROS?
If you are curious to know how frames are expressed in ROS, you can extract the coordinate frame
in which messages are expressed in by reading the value of header.frame_id variable of the
respective message.
You can also view the various frames in the robot system as a tree structure by running the
rqt_tf_tree node and pressing the TF Tree button in the GUI (make sure to zoom in).
rosrun rqt_tf_tree rqt_tf_tree
11.How to transform from quaternion to euler?
ROS uses quaternions to track and apply rotations. A quaternion has 4 components (x,y,z,w). A
quaternion is an another way to express a 3d rotation in space. You can convert a 3d orientation
from quaternion to euler angles and vice versa. You may refer to the following links for example
code:
For Python: https://answers.ros.org/question/69754/quaternion-transformations-in-python/
For C++: https://gist.github.com/marcoarruda/f931232fe3490b7fa20dbb38da1195ac
12.What the standard units of measurements used in ROS?
ROS follows the SI Unit system. For more info, refer REP103 .
13.I can not see anything in my stageros simulator.
You might need to change the view in the simulator. You can reset the view from the View menu
or change the view by clicking and dragging the mouse in the free area of the simulator window.
14.I am done with my assignment, can you recommend me a good sci-fi tv show?
Futurama, The OA, Black Mirror (Seasons 1-3), Firefly, Battlestar Galactica (have not seen this yet,
but have heard good things about it)
VIII. References:
1. ROS Tutorials: http://wiki.ros.org/ROS/Tutorials
2. Stage 4.1.1 Manual: https://codedocs.xyz/CodeFinder2/Stage/md_README.html
3. stageros Wiki: http://wiki.ros.org/stage_ros
4. rViz Wiki: http://wiki.ros.org/rviz
5. Autolab: https://autograder.cse.buffalo.edu/courses/CSE468-f19/assessments
