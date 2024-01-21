#! /usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped

SOURCE = '/base_link'
DESTINATION = '/map'
#globals()['listener'] = tf.TransformListener()

def transpose(pose: Pose):
  try:
    poseStamped = PoseStamped()
    poseStamped.header.stamp = rospy.Time.now()
    poseStamped.header.frame_id = SOURCE
    poseStamped.pose = pose
    listener=tf.TransformLister()
    #transposed = globals()['listener'].transformPose(DESTINATION, poseStamped)
    transposed = listener.transformPose(DESTINATION, poseStamped)

    return transposed
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    rospy.loginfo("could not load transform: " + SOURCE + " -> " + DESTINATION)

def lookupTransform():
    try:
        (translation, rotation) = globals()['listener'].lookupTransform(DESTINATION, SOURCE, rospy.Time(0))
        rospy.loginfo('---------')
        rospy.loginfo('Translation: ' + str(translation))
        rospy.loginfo('Rotation: ' + str(rotation))
        return (translation, rotation)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
       rospy.loginfo("could not load transform: " + SOURCE + " -> " + DESTINATION)