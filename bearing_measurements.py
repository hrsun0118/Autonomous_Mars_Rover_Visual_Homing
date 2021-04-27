#!/usr/bin/env python
""" turtlesim node: subscribe to receive turtle pose & publish bearings beta1 & beta2."""

import rospy
from move_to_midpt2l.msg import Bearings
from turtlesim.msg import Pose
import numpy as np
from math import pow, sqrt

def euclidean_distance(landmark_pose, turtle_pose_msg):
    """Euclidean distance between current landmarkX and the turtle."""
    return sqrt(pow((landmark_pose[0][0] - turtle_pose_msg.x), 2) +
                pow((landmark_pose[1][0] - turtle_pose_msg.y), 2))

def callback_turtle_pose(turtle_pose_msg):
    """Callback to receive a message from turtle pose
       Calculate beta0 & beta1"""
    global pub_bearings # ! try to understand reasons for declaring this twice

    turtle_bearings = Bearings() # [modification] custom message type needed!

    # modify the turtle_bearings steps: [modification] might be needed!
    # set landmarks l0 = [1, 1], l1 = [3,1] (note: frame range x = [0, 11.088], y = [0, 11.088], (x,y) at bottom left corner)
    l0 = np.asarray([[4.545],[5.545]])   # [modification] use mid pt for L0 & L1 to test the turtle  movement
    l1 = np.asarray([[6.545],[5.545]])
    # calculate denominators for l0 & l1
    d0 = euclidean_distance(l0, turtle_pose_msg)
    d1 = euclidean_distance(l1, turtle_pose_msg)
    # calculate numerators for l0 & l1
    n0 = l0 - np.asarray([[turtle_pose_msg.x], [turtle_pose_msg.y]]) # potential [modification] check usage of minus sign for np array
    n1 = l1 - np.asarray([[turtle_pose_msg.x], [turtle_pose_msg.y]])
    # calculate bearing
    bl = n0 / d0
    br = n1 / d1
    # put data into turtle_bearings
    turtle_bearings.beta0.x = bl[0][0]
    turtle_bearings.beta0.y = bl[1][0]
    turtle_bearings.beta1.x = br[0][0]
    turtle_bearings.beta1.y = br[1][0]

    # publish turtle_bearings
    pub_bearings.publish(turtle_bearings)   # Why can't I publish it directly?
    rospy.loginfo("bearing_left: beta0 = [[%.3f],[%.3f]]", bl[0][0], bl[1][0])   # [modification] might be needed!
    rospy.loginfo("bearing_right: beta1 = [[%.3f],[%.3f]]", br[0][0], br[1][0])   # [modification] might be needed!

def main():
    global pub_bearings
    rospy.init_node('bearing_measurements', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, callback_turtle_pose) # Can I specify "/turtleX" X = 1? Or do I need to check each time?
    pub_bearings = rospy.Publisher("bearings", Bearings, queue_size = 10)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    finally:
        # "clean up" code: executed on shutdown in case of errors. E.g. closing files/windows
        pass
