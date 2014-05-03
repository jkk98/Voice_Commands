#!/usr/bin/env python

"""
voice_cmd_vel.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.
"""

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import math
import os

from sound_play.libsoundplay import SoundClient

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion

class voice_cmd_vel:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.speed = 0.2
        self.msg = Twist()
        self.discrete_movement = False #Check to see if the robot is performing a discrete movement rather than a twist
        self.postion = (0, 0) #None
        self.goal_position = (0, 0)
        self.yaw = 0 #None
        self.goal_yaw = 0
        self.timeout = 0
        self.position_status = ""
        self.velocity_status = ""
        
        # sets up the SoundClient for playing sounds
        self.soundhandle = SoundClient()

        # publish to cmd_vel, subscribe to speech output
        self.pub_ = rospy.Publisher('cmd_vel', Twist)
        rospy.Subscriber('recognizer/output', String, self.speechCb)
        rospy.Subscriber('/odom', Odometry, self.odomCb)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if(self.discrete_movement == True):
                try:
                    self.finite_move()
                except:
                    rospy.loginfo("Failed to implement finite movement command")
                    
            try:
                self.position_status = "X: %.1f, Y: %.1f, Yaw: %.1f" %(self.position[0], self.position[1], self.yaw)
                self.velocity_status = "Forward Velocity: %.1f m/s, Angular Velocity %.1f m/s" %(self.msg.linear.x, self.msg.angular.z)
                rospy.loginfo(self.position_status)
                rospy.loginfo(self.velocity_status)
            except:
                rospy.loginfo("Could not print out status")
            self.pub_.publish(self.msg)
            r.sleep()
        
    def finite_move(self):
        angle_difference = self.goal_yaw - self.yaw
        print "angle_difference:", angle_difference
        goal_difference = math.sqrt((self.goal_position[0] - self.position[0])**2 + (self.goal_position[1] - self.position[1])**2)
        print("goal_difference:", goal_difference)
        if(angle_difference > 0.1):
            self.msg.angular.z = 0.5
        elif(angle_difference < -1):
            self.msg.angular.z = -0.1
        elif(goal_difference > 0.1):
            self.msg.linear.x = 0.5
        elif(goal_difference < -0.1):
            self.msg.linear.x = -0.5
        else:
            rospy.loginfo("Finished finite movement command")
            self.cleanup()
            pass
        
    def odomCb(self, msg):
        [r,p,yaw] = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.yaw = yaw
        self.position = (msg.pose.pose.position.x,msg.pose.pose.position.y)
        #rospy.loginfo(msg)
        
    def speechCb(self, msg):
        rospy.loginfo(msg.data)

        if msg.data.find("full speed") > -1:
            if self.speed == 0.2:
                self.msg.linear.x = self.msg.linear.x*2
                self.msg.angular.z = self.msg.angular.z*2
                self.speed = 0.4
        if msg.data.find("half speed") > -1:
            if self.speed == 0.4:
                self.msg.linear.x = self.msg.linear.x/2
                self.msg.angular.z = self.msg.angular.z/2
                self.speed = 0.2

        if msg.data.find("forward") > -1:    
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0
        elif msg.data.find("left") > -1:
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z += 0.05
            else:        
                self.msg.angular.z = self.speed*2
        elif msg.data.find("right") > -1:    
            if self.msg.linear.x != 0:
                if self.msg.angular.z > -self.speed:
                    self.msg.angular.z -= 0.05
            else:        
                self.msg.angular.z = -self.speed*2
        elif msg.data.find("back") > -1:
            self.msg.linear.x = -self.speed
            self.msg.angular.z = 0
            
        # Need to add in case statements for finite move commands
        # Will need to edit the dictionary            
            
        elif msg.data.find("stop") > -1 or msg.data.find("halt") > -1:          
            self.msg = Twist()
            ##os.system("roslaunch voice_commands test_bouncer.launch")
            
        output_message = "I heard %s." % msg.data
        ##system_message = "rosrun sound_play say.py \"I heard %s.\"" % msg.data        
        ##os.system(output_message)
        self.soundhandle.say(output_message)
        
        self.pub_.publish(self.msg)

    def cleanup(self):
        # stop the robot!
        self.discrete_movement = False
        twist = Twist()
        self.pub_.publish(twist)

if __name__=="__main__":
    rospy.init_node('voice_cmd_vel')
    try:
        voice_cmd_vel()
    except:
        pass

