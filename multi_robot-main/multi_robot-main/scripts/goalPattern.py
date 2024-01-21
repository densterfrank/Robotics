#!/usr/bin/env python3
# Implementation inspired by: https://github.com/danielsnider/follow_waypoints/blob/master/src/follow_waypoints/follow_waypoints.py

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Twist
)
from tf import TransformListener
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
def goToPose():
    #pub_gotopose1 = rospy.Publisher('/robot1/move_base_simple/goal', PoseStamped)
    #pub_gotopose2 = rospy.Publisher('/robot2/move_base_simple/goal', PoseStamped)
    map_frame = rospy.get_param("~map_frame", "map")
    robot_frame = rospy.get_param("~robot_frame", '/base_link')

    # Create target for robot1
    target_pose1 = PoseStamped()
    target_pose1.header.frame_id = map_frame
    target_pose1.header.stamp = rospy.Time.now()
    target_pose1.pose.position.x = 2.0
    target_pose1.pose.position.y = -2.0
    target_pose1.pose.position.z = 0.0
    target_pose1.pose.orientation.x = 0
    target_pose1.pose.orientation.y = 0
    target_pose1.pose.orientation.z = 0
    target_pose1.pose.orientation.w = 1

    # Create target for robot2
    target_pose2 = PoseStamped()
    target_pose2.header.frame_id = map_frame
    target_pose2.header.stamp = rospy.Time.now()
    target_pose2.pose.position.x = 2.0
    target_pose2.pose.position.y = -2.0
    target_pose2.pose.position.z = 0
    target_pose2.pose.orientation.x = 0
    target_pose2.pose.orientation.y = 0
    target_pose2.pose.orientation.z = 0
    target_pose2.pose.orientation.w = 1

    #To print the infomation of goal
    rospy.loginfo(target_pose1)
    rospy.loginfo(target_pose2)
    goal1 = MoveBaseGoal()
    goal1.target_pose = target_pose1
    goal2 = MoveBaseGoal()
    goal2.target_pose = target_pose2
    # Send goal
    # Need a little bit of a pause to let the node correctly launch
    rate = rospy.Rate(1)
    rate.sleep()
    rospy.loginfo("Publishing")
    #Calling simple action client for robot1 and robot2 
    moveBaseClient1 = actionlib.SimpleActionClient('/robot1/move_base', MoveBaseAction)
    moveBaseClient1.wait_for_server()
    moveBaseClient2 = actionlib.SimpleActionClient('/robot2/move_base', MoveBaseAction)
    moveBaseClient2.wait_for_server()
    #Sending goal1 and goal2 
    moveBaseClient1.send_goal(goal1)
    moveBaseClient2.send_goal(goal2)
    moveBaseClient1.wait_for_result()
    moveBaseClient2.wait_for_result()
    #pub_gotopose1.publish(target_pose1)
    #pub_gotopose2.publish(target_pose2)
    rospy.loginfo("Both robots reached their goals")
# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('rosbot_gotopose', anonymous=True)

        goToPose()
    except rospy.ROSInterruptException:
        pass

