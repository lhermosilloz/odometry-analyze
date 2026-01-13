import rospy
from nav_msgs.msg import Odometry
import os

def odom_cb(msg):
    z = msg.pose.pose.position.z
    os.system("clear")
    print("FAST-LIO Odometry Z")
    print("===================")
    print("position.z = " + str(z))

def main():
    rospy.init_node("fastlio_z_monitor", anonymous=True)
    rospy.Subscriber("/Odometry", Odometry, odom_cb)
    rospy.spin()

if __name__ == "__main__":
    main()