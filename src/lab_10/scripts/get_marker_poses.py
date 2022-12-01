#!/usr/bin/env python3
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
#import numpy as np
#import pdb

'''
script to receive ar_track_message & create publisher
publish position of marker(s) w.r.t. to world frame to rviz
use ros topic /ar_pose_marker
ar_track_alvar_message >> http://docs.ros.org/en/fuerte/api/ar_track_alvar/html/msg/AlvarMarker.html
http://wiki.ros.org/ar_pose 
'''

def ar_pose_callback(msg):
    #ar_poses = [] #np.zeros(len(msg))
    #for index, marker in enumerate(msg.markers):
    for marker in msg.markers:
        ar_pose = PoseStamped()
        # ------------------------------------------------------------

        # frame id
        ar_pose.header.seq = marker.header.seq
        ar_pose.header.stamp = marker.header.stamp
        ar_pose.header.frame_id = str(marker.id)
        
        #rospy.loginfo(f"\n\tar_pose.header.frame_id: {ar_pose.header.frame_id}\n\n")
        # pose
        ar_pose.pose.position.x = marker.pose.pose.position.x
        ar_pose.pose.position.y = marker.pose.pose.position.y
        ar_pose.pose.position.z = marker.pose.pose.position.z
        
        # orientation
        ar_pose.pose.orientation.x = marker.pose.pose.orientation.x
        ar_pose.pose.orientation.y = marker.pose.pose.orientation.y
        ar_pose.pose.orientation.z = marker.pose.pose.orientation.z
        ar_pose.pose.orientation.w = marker.pose.pose.orientation.w
        
        ar_pose_pub.publish(ar_pose) #publish ar pose
        # append all the poses
        #ar_poses.append(ar_pose)

        # ------------------------------------------------------------
    # rospy.loginfo(f"\n\tpublished ar_poses!\n\n{ar_poses}\n\n")
    #ar_pose_pub.publish(ar_poses) #publish ar poses
    
if __name__ == '__main__':
    try:
        rospy.init_node('get_marker_poses')
        ar_pose_pub = rospy.Publisher("/extracted_ar_poses", PoseStamped, queue_size=10) 
        rospy.Rate(20) # [Hz]
        rospy.wait_for_message("/ar_pose_marker", AlvarMarkers)
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_pose_callback) 
        rospy.spin()
    except rospy.ROSInterruptException: pass
    