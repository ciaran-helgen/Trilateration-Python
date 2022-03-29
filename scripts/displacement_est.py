#!/usr/bin/env python
from turtle import stamp
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from beacon_pub.msg import beacon
from math import sqrt
import numpy as np

from trilateration import trilateration

import matplotlib.pyplot as plt


class DisplacementEst:

    def __init__(self):

        self.beacon0 = beacon()
        self.beacon1 = beacon()
        self.beacon2 = beacon()

        # coords relative to centre of mat (1m square)
        self.c0 = np.array([0.5, 0.5, 0.00])
        self.c1 = np.array([0.5, -0.5,	0.00])
        self.c2 = np.array([-0.5, -0.5, 0.00])

        self.pose_est = PoseStamped()

        rospy.init_node('displacement_calculator', anonymous=True)

        rospy.Subscriber("/receiver0", beacon, self.cb0)
        rospy.Subscriber("/receiver1", beacon, self.cb1)
        rospy.Subscriber("/receiver2", beacon, self.cb2)
        self.pub = rospy.Publisher("/coord_rel_c0", PoseStamped, queue_size=1)

    def estPose(self):
        pose_est = PoseStamped()
        
        now = rospy.Time.now()
        pose_est.header.stamp = now
        pose_est.header.seq += 1
        pose_est.header.frame_id = "pose_est_frame"

        W = np.eye(3)  # Weigths matrix

        # Get current distance measurements, s1, s2, s3
        S = np.array([dispEst.beacon0.debug_distance, dispEst.beacon1.debug_distance, dispEst.beacon2.debug_distance]) # Distance vector
        P1 = self.c0
        P2 = self.c1
        P3 = self.c2
        P = np.array([P1, P2, P3] ) # Reference points matrix
        P = np.column_stack([P1, P2, P3])
        # estimate pose rel to beacon 0, top left of mat, and publish
        #dispEst.estPose()

        N1, N2 = trilateration(P,S,W)

        # flattening the array is probably wasteful, but makes it easier to access the solution
        N1 = N1[1:].flatten()
        N2 = N2[1:].flatten()

        # set orientation to identity quaternion
        pose_est.pose.orientation.x = 0
        pose_est.pose.orientation.y = 0
        pose_est.pose.orientation.z = 0
        pose_est.pose.orientation.w = 1

        # set x and y position
        pose_est.pose.position.x = N1[0]
        pose_est.pose.position.y = N1[1]
        # set z value to valid solution (could just use abs())
        if N1[2] > 0:
            pose_est.pose.position.z = N1[2]
        else:
            pose_est.pose.position.z = N2[2]

        # Populate pose_est here ^^^
        return pose_est

    def cb0(self, data):
        self.beacon0 = data
        #rospy.loginfo("Dist 0: %f", self.beacon0.debug_distance)
    
    def cb1(self, data):
        self.beacon1 = data
        #rospy.loginfo("Dist 1: %f", self.beacon1.debug_distance)
    
    def cb2(self, data):
        self.beacon2 = data
        #rospy.loginfo("Dist 2: %f", self.beacon2.debug_distance)


if __name__ == '__main__':
    dispEst = DisplacementEst()
    # publish estimate at 1 Hz
    rate = rospy.Rate(1)

    # Loop to publish pose estimate. Subscriber callbacks continue in background
    while not rospy.is_shutdown():
        try:
           
            pose_est = dispEst.estPose()
            dispEst.pub.publish(pose_est)

            rate.sleep()
        except rospy.ROSInterruptException:
            pass