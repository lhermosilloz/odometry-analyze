#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import os
import tf.transformations as tft
import math

def odom_cb(msg):
    # Position
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

    # Orientation (Quaternion)
    q = msg.pose.pose.orientation
    quat = [q.x, q.y, q.z, q.w]

    # Convert quaternion to roll, pitch, and yaw
    roll, pitch, yaw = tft.euler_from_quaternion(quat)

    # Convert to degrees
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)

    os.system("clear")
    print("FAST-LIO Odometry Z")
    print("===================")
    print("position.x = " + str(x))
    print("position.y = " + str(y))
    print("position.z = " + str(z))
    print("===================")
    print("roll = " + str(roll_deg))
    print("pitch = " + str(pitch_deg))
    print("yaw = " + str(yaw_deg))

def main():
    rospy.init_node("fastlio_z_monitor", anonymous=True)
    rospy.Subscriber("/Odometry", Odometry, odom_cb)
    rospy.spin()

if __name__ == "__main__":
    main() 