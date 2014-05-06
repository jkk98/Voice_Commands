#!/usr/bin/env python

"""
voice_cmd_vel.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.
"""

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import os

import math
from math import *
from sound_play.libsoundplay import SoundClient

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import *
from sensor_msgs.msg import *
from tf.transformations import euler_from_quaternion

class voice_cmd_control:
    number = {'zero': 0, 'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5, 'six': 6, 'seven': 7, 'eight': 8, 'nine': 9}
    nodes = ["test_bouncer.launch", "wall_following.launch"]

    def __init__(self):
        print("====== Beginning Init ========")
        rospy.on_shutdown(self.cleanup)
        self.speed = 0.2
        self.msg = Twist()
        self.discrete_movement = False #Check to see if the robot is performing a discrete movement rather than a twist
        self.pfield = False #Check to see if robot is undergoing a pfield action to a goal
        self.node_launch = False #Check to see if a node is currently launched
        self.current_launched_node = ""
        self.obstacle_force = [0,0]
        self.position = [0, 0] #None
        self.goal_position = [0, 0]
        self.yaw = 0 #None
        self.goal_yaw = 0
        self.goal_threshold = 1
        self.finite_move_distance = .5
        self.finite_angle_distance = math.pi / 3
        self.timeout = 0
        self.position_status = ""
        self.velocity_status = "" 
        self.goal_status = ""
            
        
        #Subscribe to the laser scan topic
        rospy.Subscriber("base_scan", LaserScan, self.laserCallback)         
        
        # sets up the SoundClient for playing sounds
        self.soundhandle = SoundClient()

        # publish to cmd_vel, subscribe to speech output
        self.pub = rospy.Publisher('cmd_vel', Twist)
        rospy.Subscriber('recognizer/output', String, self.speechCb)
        rospy.Subscriber('/odom', Odometry, self.odomCb)
        print(" ==== Finished initializing ====")

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if(self.discrete_movement == True):
                try:
                    self.finite_move()
                except:
                    print("Failed to implement finite movement command")
                    
            if(self.pfield == True):
                print("running pfield")
                self.pfield_goal()
                    
            try:
                self.position_status = "X: %.1f, Y: %.1f, Yaw: %.1f" %(self.position[0], self.position[1], self.yaw)
                self.goal_status = "Goal X: %.1f, Goal Y: %.1f, Goal Yaw: %.1f" %(self.goal_position[0], self.goal_position[1], self.goal_yaw) 
                self.velocity_status = "Forward Velocity: %.1f m/s, Angular Velocity %.1f m/s" %(self.msg.linear.x, self.msg.angular.z)
                #rospy.loginfo(self.position_status)
                #rospy.loginfo(self.velocity_status)
            except:
                rospy.loginfo("Could not get status")
            self.pub.publish(self.msg)
            r.sleep()
        
    def finite_move(self):
        angle_difference = self.goal_yaw - self.yaw
        print "angle_difference:", angle_difference
        goal_difference = math.sqrt((self.goal_position[0] - self.position[0])**2 + (self.goal_position[1] - self.position[1])**2)
        print("goal_difference:", goal_difference)
        if(angle_difference > 0.1):
            self.msg.angular.z = 0.5
        elif(angle_difference < -.1):
            self.msg.angular.z = -0.5
        elif(goal_difference > 0.1):
            self.msg.linear.x = 0.5
        elif(goal_difference < -0.1):
            self.msg.linear.x = -0.5
        else:
            rospy.loginfo("Finished finite movement command")
            self.stop()
            pass
    
    # add_forces command based on pfield code
    def add_forces(self, a, b):
        assert len(a) == len(b), "Force vectors differ in length"
        c = [a[i] + b[i] for i in range(len(a))]
        return c

    def wrap_angle(self, angle):
        #This function will take any angle and wrap it into the range [-pi, pi]
        while angle >= math.pi:
            angle = angle - 2*math.pi
            
        while angle <= -math.pi:
            angle = angle + 2*math.pi
        return angle

    def goal_force(self,angle_to_goal):
        #This function determines and returns the attractive force [s_x,s_y] to the goal, given the angle to the goal
    
        #Parameter : MODIFY
        strength = 2.0 #This is the magnitude of the attractive goal force
        #End of Parameters
    
        force_angle = self.wrap_angle(angle_to_goal - self.yaw)
        s_x = strength * cos(force_angle)
        s_y = strength * sin(force_angle)
    
        return [s_x,s_y]

    def drive_from_force(self, force,twist):
        #This function takes in a force [x,y] and determines the drive command (Twist) that should be sent to the robot motors
        global robot
    
        print 'DRIVE FORCE : ', force
    
        turn_multiplier = 0.5 #This is multiplied by the angle difference to get the turn command
        spin_threshold = math.pi / 2 #If the difference between the robot's yaw and the force direction is greater than this, we only spin
        drive_multiplier = 0.8 #This is multiplied by the magnitude of the force vector to get the drive forward commnd
    
        #Determine the angle of the force
        force_angle = atan2(force[1],force[0])
        force_mag = hypot(force[0],force[1])
    
        #Get difference to robot's current yaw
        a_diff = self.wrap_angle(force_angle)
    
        #Get turn speed
        twist.angular.z = turn_multiplier * a_diff
    
        #Do we just spin
        if abs(a_diff) < spin_threshold:
            twist.linear.x = drive_multiplier * force_mag
    
        return twist
        
    def laserCallback(self, data):
        #This function looks at the current laser reading and computes the obstacle avoidance force vector
        #which is set in the global variable 'obstacle force'
    
        global obstacle_force  #format = [x,y]
        global robot  #format = [x_position, y_position, yaw]
        
        #Make sure this is zeroed out before beginning
        self.obstacle_force = [0,0]
    
        #Parameters: MODIFY
        strength = 3.0 #This is the magnitude of the repulsive force from each individual laser scan data point
        obstacle_distance_threshold = 1.0 #How close to the obstacle do we have to be to feel repulsed
        #End of Parameters
    
        cur_angle = data.angle_min #cur_angle will always have the relative angle between the robot's yaw and the current laser reading
    
        for i in range(len(data.ranges)):
            if data.ranges[i] < obstacle_distance_threshold:
                force_direction = self.wrap_angle(cur_angle + math.pi)
                strength_x = strength*((obstacle_distance_threshold - data.ranges[i])/obstacle_distance_threshold)* cos(force_direction)
                strength_y = strength*((obstacle_distance_threshold - data.ranges[i])/obstacle_distance_threshold)* sin(force_direction)
    
                self.obstacle_force = self.add_forces(self.obstacle_force,[strength_x, strength_y]) 
    
            cur_angle = cur_angle + data.angle_increment
            
    def pfield_goal(self):
        print(self.position, self.goal_position)
        if math.hypot(self.position[0]-self.goal_position[0],self.position[1]-self.goal_position[1]) > self.goal_threshold:
            twist = Twist()
            
            #Get vector from robot to goal
            angle_to_goal = atan2(self.goal_position[1] - self.position[1],self.goal_position[0] - self.position[0])
            print("angle to goal", angle_to_goal)

            #Compute attractive force to goal
            g_force = self.goal_force(angle_to_goal)
            print("g_force", g_force)
            #Add to obstacle avoidance forces
            total_force = self.add_forces(g_force,self.obstacle_force)
            print("total force", total_force)
            #Get final drive command from total force
            twist = self.drive_from_force(total_force,twist) 

            #Publish drive command, then sleep
            #pub.publish(twist)
            #rospy.sleep(0.1)
            self.msg = twist        
        else:
            print 'I THINK THE ROBOT IS AT THE GOAL'
        
    def odomCb(self, msg):
        [r,p,yaw] = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.yaw = yaw
        self.position = [msg.pose.pose.position.x,msg.pose.pose.position.y]
        #rospy.loginfo(msg)
        
    def speechCb(self, msg):
        rospy.loginfo(msg.data)
        output_message = "I heard %s." % msg.data

        if msg.data.find("full") > -1:
            if self.speed == 0.2:
                self.msg.linear.x = self.msg.linear.x*2
                self.msg.angular.z = self.msg.angular.z*2
                self.speed = 0.4
            if(self.pfield == False):
                print("Beginning pfields perhaps")
                self.pfield = True
                self.msg = Twist()
        if msg.data.find("half") > -1:
            if self.speed == 0.4:
                self.msg.linear.x = self.msg.linear.x/2
                self.msg.angular.z = self.msg.angular.z/2
                self.speed = 0.2
            if(self.node_launch == False):
                os.system("roslaunch voice_commands wall_following.launch &")
                self.node_launch == True

        if msg.data.find("twist forward") > -1:    
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0
        elif msg.data.find("twist left") > -1:
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z += 0.05
            else:        
                self.msg.angular.z = self.speed*2
        elif msg.data.find("twist right") > -1:    
            if self.msg.linear.x != 0:
                if self.msg.angular.z > -self.speed:
                    self.msg.angular.z -= 0.05
            else:        
                self.msg.angular.z = -self.speed*2
        elif msg.data.find("twist back") > -1:
            self.msg.linear.x = -self.speed
            self.msg.angular.z = 0
            
        # Discrete Movements
        elif (msg.data.find("move four") > -1) or (msg.data.find("move forward") > -1):
            self.goal_yaw = self.yaw # reset the goal yaw
            self.goal_position[0] = self.position[0] + cos(self.wrap_angle(self.yaw)) * self.finite_move_distance
            self.goal_position[1] = self.position[1] + sin(self.wrap_angle(self.yaw)) * self.finite_move_distance
            self.discrete_movement = True
#        elif msg.data.find("move back") > -1:
#            self.goal_yaw = self.yaw # reset the goal yaw
#            self.goal_position[0] = -cos(self.wrap_angle(self.yaw)) * self.finite_move_distance
#            self.goal_position[1] = -sin(self.wrap_angle(self.yaw)) * self.finite_move_distance
#            self.discrete_movement = True
        elif msg.data.find("rotate right") > -1:
            self.goal_position = self.position # reset the goal
            self.goal_yaw = self.yaw - self.finite_angle_distance
            self.discrete_movement = True
        elif msg.data.find("rotate left") > -1:
            self.goal_position = self.position # reset the goal
            self.goal_yaw = self.yaw + self.finite_angle_distance
            self.discrete_movement = True
        
            
        # Need to add in case statements for finite move commands
        # Will need to edit the dictionary            
            
        elif msg.data.find("stop") > -1 or msg.data.find("halt") > -1:          
            #self.cleanup()
            self.msg = Twist()
            self.stop()

        elif ((msg.data.split())[0] == "x"):
            msg_list = msg.data.split()
            try:
                param = msg_list[1].strip()
                print(param)
                value = voice_cmd_control.number[param]
                print(value)
                self.goal_position[0] = value #self.get_point(msg_list[1])
                print("goal position:", self.goal_position)
            except Exception as e:
                print(e)
                print("second paramater is not an int")
                
        elif ((msg.data.split())[0] == "y"):
            msg_list = msg.data.split()
            try:
                param = msg_list[1].strip()
                print(param)
                value = voice_cmd_control.number[param]
                print(value)
                self.goal_position[1] = value #self.get_point(msg_list[1])
                print("goal position:", self.goal_position)
            except Exception as e:
                print(e)
                print("second paramater is not an int") 
            
        elif (msg.data.find("goto")):
            print "==== Status ===="
            print self.position_status
            print self.velocity_status
            print self.goal_status
            #output_message = self.position_status + self.velocity_status + self.goal_status       
            
        elif msg.data.find("exit") > -1:
            exit()
            
        ##system_message = "rosrun sound_play say.py \"I heard %s.\"" % msg.data        
        ##os.system(output_message)
        self.soundhandle.say(output_message)
        
        #self.pub_.publish(self.msg)

    def stop(self):
        self.discrete_movement = False
        self.pfield = False
        #if(self.node_launch == True):
        os.system("rosnode kill /wall_following")
        self.msg = Twist()
        self.pub.publish(self.msg)

    def cleanup(self):
        # stop the robot!
        if(self.node_launch == True):
            os.system("rosnode kill /test_bouncer")
        self.twist = Twist()
        self.pub.publish(self.twist)

if __name__=="__main__":
    rospy.init_node('voice_control')
    control = voice_cmd_control()
#    try:
#        voice_cmd_control()
#    except Exception as e:
#            print(e)
        

