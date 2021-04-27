#!/usr/bin/env python
""" bearing_controller node: subscribe to recceive bearings & turtle pose, and publish cmd_vel."""

import rospy
from move_to_midpt2l.msg import Bearings
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import cos, acos, pow, sqrt, fmod, pi
import numpy as np

pose = Pose()
pose.theta = 0

def callback_bearings(msg):
    """Callback to receive a message from bearings"""
    global pub_cmd_vel
    global pose

    # obtain u
    beta0_star = np.asarray([[-1], [0]])
    beta1_star = np.asarray([[1], [0]])
    u = (np.asarray([[msg.beta0.x],[msg.beta0.y]]) \
        + np.asarray([[msg.beta1.x],[msg.beta1.y]])) \
        - (beta0_star + beta1_star)

    # calculate theta_u
    abs_u = sqrt(pow(u[0][0], 2) + pow(u[1][0], 2)) # calculate abs_u
    cos_theta_u = u[0][0] / abs_u
    sin_theta_u = u[1][0] / abs_u   # Q: 2 answers for sin_theta_u:
                                    # vector_1 X vector_u = (1,0) X (u1, u2) = u2 - I chose this one - why?
                                    # vector_u X vector_1 = (u1, u2) X (1,0) = -u2
    if sin_theta_u >= 0:
        theta_u = acos(cos_theta_u)
    else:
        theta_u = -acos(cos_theta_u)

    # calculate theta_e
    theta_e = pose.theta - theta_u # if I add a "-"(negative sign), then turtle move backwards
    # theta_e=fmod(theta_e+pi,2*pi)-pi # fmod(x,y): returns remainder of x/y as a float
        # Previous fmod fn doesn't work. It's bound is outside my desired theta_e bound: [-2pi, pi] instead of [-pi, pi]
        # It gets stuck at u = [-1, -1]
    if theta_e > pi:
        theta_e = theta_e - 2 * pi
    elif theta_e < -pi:
        theta_e = theta_e + 2 * pi
    rospy.loginfo("u =  [%.3f][%.3f], cos=%.3f, sin=%.3f, theta_u = %.3f, theta_e= %.3f", u[0][0], u[1][0], cos_theta_u, sin_theta_u, theta_u * 180.0 / pi, theta_e)

    # set phi = 1 & then adjust
    phi = 1.5
    # calculate w
    if theta_e <= 0:
        w = phi
    else:
        w = -phi
    # calculate v
    v = cos(theta_e) * abs_u

    # initialize & obtain cmd_vel
    cmd_vel = Twist()
    # linear velocity
    cmd_vel.linear.x = v # [reset back to "v" later] / test 0.0
    cmd_vel.linear.y = 0.0
    cmd_vel.linear.z = 0.0
    # angular velocity
    cmd_vel.angular.x = 0.0
    cmd_vel.angular.y = 0.0
    cmd_vel.angular.z = w

    # publish w & v
    pub_cmd_vel.publish(cmd_vel) # [uncomment later]


    # log info
    #rospy.loginfo("cmd/vel:")  # uncomment later
    #rospy.loginfo("Linear: x = %.3f, y = %.3f, z = %.3f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z)  # uncomment later
    #rospy.loginfo("Angular: x = %.3f, y = %.3f, z = %.3f",cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z)  # uncomment later

def callback_turtle_pose(msg):
    global pose
    pose.theta = msg.theta  # get turtle pose & theta
    # rospy.loginfo("New Pose obtained. Theta = %.3f", pose.theta)

def main():
    global pub_cmd_vel
    global pose

    rospy.init_node('bearing_controller', anonymous=False)
    rospy.Subscriber('bearings', Bearings, callback_bearings) # obtain bearings
    rospy.Subscriber('/turtle1/pose', Pose, callback_turtle_pose) # modify global variable "pose"
    # Or do I send the turtle pose along with bearings from bearing_measurements node?
    # Or should I callback_turtle_pose whenever I got a new bearing? - this one for now
    pub_cmd_vel = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 10)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    finally:
        # "clean up" code: executed on shutdown in case of errors. E.g. closing files/windows
        pass
