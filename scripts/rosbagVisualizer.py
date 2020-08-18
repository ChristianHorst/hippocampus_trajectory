#!/usr/bin/env python
import numpy as np

from pyquaternion import Quaternion
import rospy
import tf
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.msg import  HippocampusControl,HippocampusDesired,AttitudeControlExt,HippocampusOutput
class rosbag:

    def __init__(self):
        self.position  = [0.0, 0.0, 0.0]

        rospy.Subscriber("/uuv00/pose_px4", PoseStamped, self.poseCallback)
        rospy.Subscriber("hippocampus/output", HippocampusOutput, self.output_callback)
        self.desired_pub = rospy.Publisher("/hippocampus/desired", HippocampusDesired, queue_size=1)

    def poseCallback(self,pose):
        print("poseCallback")
    def output_callback(self, output):
        print("outputCallback")


    def main_function(self):
        print("main")


def main():
    rospy.init_node('rosbag_visual')
    rate = rospy.Rate(50)
    rb=rosbag()
    while not rospy.is_shutdown():
        rb.main_function()
        rate.sleep()

if __name__ == '__main__':
    main()