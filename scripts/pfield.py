#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion
import math
from math import *

robot = [0,0,0]

def add_forces(a, b):
    assert len(a) == len(b), "Force vectors differ in length"
    c = [a[i] + b[i] for i in range(len(a))]
    return c


def drive_from_force(force,twist):
    global robot #format [x_pos, y_pos, yaw]
    
    magnitude = hypot(force[0], force[1])
    angle = atan2(force[0], force[1])
    print(angle)
    print(magnitude)
    
    twist.linear.x = magnitude
    twist.angular.z = angle
    #force_angle = atan2()
    
    #Determine drive command here
    # magnitude = hypot(x,y)

    return twist
    
    
def drive_to_goal(force, twist):
    global robot #format [x_pos, y_pos, yaw]
    
    #Determine drive command here
    
    twist = Twist()
    twist.angular.z = 0
    twist.linear.x = 1

    goal_angle = atan2((-goal_x + robot[0]), (-goal_y + robot[1]))
    print("Goal Angle:", goal_angle)
    if(abs(goal_angle - robot[2]) > .1):
        twist.angular.z = (goal_angle - robot[2]) / 2
    #twist.angular.z = goal_angle
    twist.linear.x = 1

    return twist

def robotCallback(data):
    global robot
    [r,p,yaw] = euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
    robot = [data.pose.pose.position.x,data.pose.pose.position.y,yaw]

if __name__ == '__main__':
    rospy.init_node('lec3', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.Subscriber("base_pose_ground_truth", Odometry, robotCallback)

    global robot
    goal_x = -10
    goal_y = 16
    

    while not rospy.is_shutdown():
        twist = Twist()
        twist.angular.z = 0
        twist.linear.x = 1
       
        #Determine Force
        total_force = [0,0]
        
        ##force = [0, 0]
        #total_force = add_forces((robot[0], robot[1]), (25, 25))
        print(total_force)        
        
        twist = drive_from_force(total_force, twist)
        #twist = drive_to_box(total_force,twist)
        

        pub.publish(twist)
        rospy.sleep(.1)
        


    print 'Done'

    twist = Twist()
    pub.publish(twist)

