#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *

obstacle_near = False

def laserCallback(data):
    global obstacle_near

    obstacle_distance_threshold = 2.0

    print 'Got new laser scan at ', rospy.Time.now()
    min_range = data.range_max
    for i in range(len(data.ranges)):
        if data.ranges[i] < min_range:
            min_range = data.ranges[i]

    print 'Minimum range from scan is : ', min_range
    if min_range < obstacle_distance_threshold:
        obstacle_near = True
    else:
        obstacle_near = False


if __name__ == '__main__':
    rospy.init_node('bouncer', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.Subscriber("base_scan", LaserScan, laserCallback) 

    global obstacle_near

    DRIVE_STATE = 0
    SPIN_STATE = 1

    current_state = SPIN_STATE

    while not rospy.is_shutdown():
        twist = Twist()
        
        if current_state == DRIVE_STATE:
            #Behavior
            twist.linear.x = 1.0
            #Transition
            if obstacle_near:
                current_state = SPIN_STATE
        elif current_state == SPIN_STATE:
            #Behavior
            twist.angular.z = 0.7
            #Transition
            if not obstacle_near:
                current_state = DRIVE_STATE

        pub.publish(twist)
        rospy.sleep(0.1)

    print 'Done'

    twist = Twist()
    pub.publish(twist)

