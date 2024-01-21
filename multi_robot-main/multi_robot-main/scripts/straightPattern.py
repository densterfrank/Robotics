#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.data)

def straightPattern():
    rospy.init_node('straightPattern', anonymous=True)
    #simple cmd_vel for robot1 and robot2
    cmd_vel_pub1 = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
    cmd_vel_pub2 = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=10)
    #cmd_vel_pub3 = rospy.Publisher('/robot3/cmd_vel', Twist, queue_size=10)
    pub = rospy.Publisher('common_topic', String, queue_size=10)
    

    linear_velocity = 0.2  # Linear velocity of the robots
    
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():

        
        # Calculate the angle to the target position
        
        
        # Create a Twist message with linear velocity towards the target and zero angular velocity
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = 0.0
        
        # Publish the Twist message
        cmd_vel_pub1.publish(cmd_vel)
        cmd_vel_pub2.publish(cmd_vel)
        #cmd_vel_pub3.publish(cmd_vel)

        rate.sleep()

if __name__ == '__main__':
    try:
        straightPattern()
    except rospy.ROSInterruptException:
        pass
