#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from lora_messages.msg import lora_distance
import numpy as np

from trilateration import trilateration

class DisplacementEst:

    def __init__(self):

        self.dist0 = 0.000
        self.dist1 = 0.000
        self.dist2 = 0.000

        # coords relative to centre of mat (1m square)
        self.c0 = np.array([0.5, 0.5, 0.00])
        self.c1 = np.array([0.5, -0.5,	0.00])
        self.c2 = np.array([-0.5, -0.5, 0.00])

        self.pose_est = PoseStamped()

        rospy.init_node('rel_pose_calculator', anonymous=False)

        rospy.Subscriber("/distance0", lora_distance, self.cb0)
        rospy.Subscriber("/distance1", lora_distance, self.cb1)
        rospy.Subscriber("/distance2", lora_distance, self.cb2)

        self.pub = rospy.Publisher("/trilat_pose_est", PoseStamped, queue_size=1)

    def estPose(self):
        self.pose_est = PoseStamped()
        
        now = rospy.Time.now()
        self.pose_est.header.stamp = now
        self.pose_est.header.seq += 1
        self.pose_est.header.frame_id = "mat_frame"

        W = np.eye(3)  # Weights matrix

        # Get current distance measurements, s1, s2, s3
        S = np.array([self.dist0, self.dist1, self.dist2]) # Distance vector
        # coords of antennae rel to centre of mat
        P1 = self.c0
        P2 = self.c1
        P3 = self.c2
        P = np.array([P1, P2, P3] ) # Reference points matrix
        P = np.column_stack([P1, P2, P3])
        
        N1, N2 = trilateration(P,S,W)

        # flattening the array is probably wasteful, but makes it easier to access the solution
        N1 = N1[1:].flatten()
        N2 = N2[1:].flatten()

        # set orientation to identity quaternion. The algorithm can't solve orientation!
        self.pose_est.pose.orientation.x = 0
        self.pose_est.pose.orientation.y = 0
        self.pose_est.pose.orientation.z = 0
        self.pose_est.pose.orientation.w = 1

        # set x and y position
        self.pose_est.pose.position.x = N1[0]
        self.pose_est.pose.position.y = N1[1]
        # set z value to valid solution (could just use abs())
        if N1[2] > 0:
            self.pose_est.pose.position.z = N1[2]
        else:
            self.pose_est.pose.position.z = N2[2]

        #return the pose estimate
        return self.pose_est

    def cb0(self, data):
        self.dist0 = data.distance
        #rospy.loginfo("Dist 0: %f", self.beacon0.debug_distance)
    
    def cb1(self, data):
        self.dist1 = data.distance
        #rospy.loginfo("Dist 1: %f", self.beacon1.debug_distance)
    
    def cb2(self, data):
        self.dist2 = data.distance
        #rospy.loginfo("Dist 2: %f", self.beacon2.debug_distance)


if __name__ == '__main__':
    dispEst = DisplacementEst()
    br = tf.TransformBroadcaster()
    static_br = tf.TransformBroadcaster()
    # publish estimate at 1 Hz
    rate = rospy.Rate(1)

    # Loop to publish pose estimate. Subscriber callbacks continue in background
    while not rospy.is_shutdown():
        try:
            
            dispEst.estPose()
            dispEst.pub.publish(dispEst.pose_est)

            # Publish mat-pose estimate transform
            br.sendTransform((dispEst.pose_est.pose.position.x, 
                        dispEst.pose_est.pose.position.y, 
                        dispEst.pose_est.pose.position.z),
                        [ dispEst.pose_est.pose.orientation.x,
                        dispEst.pose_est.pose.orientation.y, 
                        dispEst.pose_est.pose.orientation.z, 
                        dispEst.pose_est.pose.orientation.w ],
                        rospy.Time.now(),
                        "trilat_drone_pose_frame",
                        "mat_frame")
            # Publish static transform from world frame to mat
            static_br.sendTransform((0, 0, 0),
                        [0, 0, 0, 1],
                        rospy.Time.now(),
                        "mat_frame",
                        "world")

            rate.sleep()
        except rospy.ROSInterruptException:
            pass