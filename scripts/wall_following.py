#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import math
import random

#Global variable declarations
front_distance = -1.0
wall_distance = -1.0

def laserCallback(laser_data):
#This function will set front_distance and wall_distance.  Both of these work in the same way.  If something is closer than the detection_distance for that sensor, the value will tell how close something is.  If nothing is closer than the detection distance, the value of -1 will be used.

    global front_distance
    global wall_distance

    #Detection distance threshold
    front_detection_distance = 0.9
    wall_detection_distance = 0.9

    wf_min_right = 10.0
    wf_min_front = 10.0
    
    cur_angle = laser_data.angle_min 

    for i in range(len(laser_data.ranges)):
        if abs(cur_angle + math.pi/2) < (math.pi / 8):
            #Wall sensor
            if laser_data.ranges[i] < wf_min_right:
                wf_min_right = laser_data.ranges[i]

        if abs(cur_angle) < (math.pi / 8):
            if laser_data.ranges[i] < wf_min_front:
                wf_min_front = laser_data.ranges[i]

        cur_angle = cur_angle + laser_data.angle_increment

    front_distance = -1
    wall_distance = -1
    if wf_min_front < front_detection_distance:
        front_distance = wf_min_front
    if wf_min_right < wall_detection_distance:
        wall_distance = wf_min_right

if __name__ == '__main__':
    rospy.init_node('lab3', anonymous=True) #Initialize the ros node
    pub = rospy.Publisher('cmd_vel', Twist) #Create our publisher to send drive commands to the robot
    rospy.Subscriber("base_scan", LaserScan, laserCallback) #Subscribe to the laser scan topic

    global front_distance  #How close is something in front of the robot (-1 = nothing is closer than the threshold)
    global wall_distance   #How close is something on the right side of the robot (-1 = nothing is closer than the threshold)

    #####LAB 3 BEGIN CODE##############

    ###YOUR CODE HERE##################
    #This is where you should implement the code that will create the wall following behavior, using
    #the sensor inputs front_distance and wall_distance
    #You are free to add helper functions, variable definitions, whatever you need to get the job done

    state = "Explore"

    while not rospy.is_shutdown():
        twist = Twist()
        print("Front Distance:", front_distance)
        print("Wall Distance:", wall_distance)
        
        """ Finite State Machine """
        ## Note that this implementation of the FSM is actually redundant in areas
        ## and as such I did not even bother adding some extra stages as my 
        ## Follow Wall stage acts a case switch for the different adjustments
        if(state == "Explore"):
            if((front_distance == -1) and (wall_distance == -1)):
                twist.linear.x = 1
                twist.angular.z = random.uniform(-math.pi, math.pi)
                state = "Explore"
            if((front_distance == -1) and (wall_distance != -1)):
                twist.linear.x = .1
                state = "Follow Wall"
                print("Entering State: Follow Wall")
            if((front_distance != -1) and (wall_distance == -1)):
                twist.angular.z = .5
                state = "Turn Left to Wall"
                print("Entering State: Turn to Wall")
        if(state == "Follow Wall"):
             if((front_distance == -1) and (wall_distance != -1)):
                twist.linear.x = 1
                state = "Follow Wall"
                if(wall_distance > 0.8):
                    twist.angular.z = -.5
                    twist.linear.x = .1
                if(wall_distance < .25):
                    twist.angular.z = .25
                    twist.linear.x = .5
             elif(front_distance != -1):
                 twist.angular.z = .5
                 state = "Turn Left to Wall"
             elif(wall_distance == -1):
                 twist.angular.z = -.5
                 twist.linear.x = .5
                 state = "Follow Wall"
        if(state == "Turn Left to Wall"):
             if((front_distance != -1)):
                twist.angular.z = .5
                state = "Turn Left to Wall"
             else:
                 twist.linear.x = .1
                 state = "Follow Wall"
            
        
        #Publish drive command, then sleep
        pub.publish(twist)
        rospy.sleep(0.1)

    ######LAB 3 END CODE#################

    print 'Done'

    twist = Twist()
    pub.publish(twist)

