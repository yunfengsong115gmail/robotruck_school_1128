#!/usr/bin/env python3
# -*- coding: utf-8 -*
"""
最简单的控制器，让小车以固定转角、固定油门运动

"""
import rospy


from math import *

from std_msgs.msg import String

from robominer_msgs.msg import *
from robominer_msgs.srv import *

from visualization_msgs.msg import Marker

from tf.broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


def move_truck():

    rospy.init_node('truck_control_node')
 
    cmd_pub = rospy.Publisher('/guardian/control_cmd', ControlCmd, queue_size=1)
    cmd_aux_pub = rospy.Publisher('/guardian/control_cmd_aux', ControlCmdAux, queue_size=1)
    rate = rospy.Rate(30)
 
 
    while not rospy.is_shutdown():
        controlCmd = ControlCmd()
        controlCmd.header.stamp = rospy.Time.now()
        controlCmd.target_drive_mode.mode = 2
        controlCmd.target_throttle = 40.0
        controlCmd.target_brake = 0.0
        controlCmd.target_steering = 30.0
        controlCmd.target_gear.gear = Gear.D
        cmd_pub.publish(controlCmd)

        controlCmdAux = ControlCmdAux()
        controlCmdAux.header.stamp = rospy.Time.now()
        controlCmdAux.target_epb.epb = 0
        cmd_aux_pub.publish(controlCmdAux)
 
        rate.sleep()
 
if __name__ == '__main__':
    try:
        move_truck()
    except rospy.ROSInterruptException:
        pass