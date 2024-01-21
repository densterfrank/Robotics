#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist

disToObstacle = 0.5

def callback(msg): 
  rospy.loginfo(rospy.get_caller_id() + " The distance to obstacle is -  %s",msg.ranges[0]) #prints on terminal
 

      
    
  move.linear.x = 0.2
  

  pub.publish(move)
  

rospy.init_node('sub_node')
sub = rospy.Subscriber('/scan', LaserScan, callback) #We subscribe to the laser's topic
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
rate = rospy.Rate(2)
move = Twist()


rospy.spin()