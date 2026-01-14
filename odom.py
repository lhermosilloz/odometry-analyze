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

    # Convert FLU to FRD
    q_x180 = [1.0, 0.0, 0.0, 0.0]
    q_frd = tft.quaternion_multiply(quat, q_x180)

    # Convert quaternion to roll, pitch, and yaw
    frd_roll, frd_pitch, frd_yaw = tft.euler_from_quaternion(q_frd)

    # Convert FRD to degrees
    frd_roll_deg = math.degrees(frd_roll)
    frd_pitch_deg = math.degrees(frd_pitch)
    frd_yaw_deg = math.degrees(frd_yaw)

    # Rotation matrix (3x3)
    R = tft.quaternion_matrix(quat)[:3, :3]

    # Body axes expressed in world frame (columns of R)
    x_w = R[:, 0]   # body +X in world
    y_w = R[:, 1]   # body +Y in world
    z_w = R[:, 2]   # body +Z in world

    # Dot product with NED world basis vectors
    z_points_down = z_w[2]
    y_points_right = y_w[1]
    x_points_fwd = x_w[0]

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
    print("===================")
    print("FRD roll = " + str(frd_roll_deg))
    print("FRD pitch = " + str(frd_pitch_deg))
    print("FRD yaw = " + str(frd_yaw_deg))
    print("===================")
    print("x_w = " + str(x_w))
    print("y_w = " + str(y_w))
    print("z_w = " + str(z_w))
    print("===================")
    print("z_points_down = " + str(z_points_down))
    print("y_points_right = " + str(y_points_right))
    print("x_points_fwd = " + str(x_points_fwd))


def main():
    rospy.init_node("fastlio_z_monitor", anonymous=True)
    rospy.Subscriber("/Odometry", Odometry, odom_cb)
    rospy.spin()

if __name__ == "__main__":
    main()