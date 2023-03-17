#!/usr/bin/env python3
#-*- coding: utf-8 -*-
from imu_driver.srv import convert_to_quaternion, convert_to_quaternionResponse
import rospy
import math


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return qx, qy, qz, qw


def handle_convert_to_quaternion(req):
    roll = req.roll
    pitch = req.pitch
    yaw = req.yaw

    x, y, z, w = quaternion_from_euler(roll, pitch, yaw)

    response = convert_to_quaternionResponse()
    response.x = x
    response.y = y
    response.z = z
    response.w = w

    return response


if __name__ == '__main__':
    rospy.init_node('convert_to_quaternion_server')
    s = rospy.Service('convert_to_quaternion', convert_to_quaternion, handle_convert_to_quaternion)
    rospy.loginfo("Ready to convert Euler angles to quaternions.")
    rospy.spin()

