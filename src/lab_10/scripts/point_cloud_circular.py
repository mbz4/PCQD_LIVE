#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import numpy as np
#import pdb
from lab_10.msg import Obsposelist # import Obsposelist message
h = 1.0
radius = 0.4
pcl_pub = None

def create_a_circle(center_x,center_y):
    points = np.arange(0, 2*np.pi, np.pi/180)
    x_points = center_x + radius*np.cos(points) #get list of  x coordinates which is on the circumference of circle with center center_x 
    y_points = center_y + radius*np.sin(points) #get list of  y coordinates which is on the circumference of circle with center center_y 
    z_points = h*np.ones(len(x_points)) #get the list z_points , in our case z points represent constant height h ,defined above .This list should be of same lenght of x_points      #points [[x1,y1,z1],[x2,y2,z2]......[xn,yn,zn]]
    return np.vstack((x_points, y_points, z_points)).T

def publish_pointcloud(obs_pose):
    global pcl_pub     
    list_circular_point_list = [] #this list will have  all the points that needs to be shown as a point cloud
    # iterate through the obs_pose list 
    for i in range(len(obs_pose.obs_poses_list)):
        center_x = obs_pose.obs_poses_list[i].transform.translation.x # get the x coordinate of the obstacle
        center_y = obs_pose.obs_poses_list[i].transform.translation.y # get tthe y coordinate of the obstacle
        circle = create_a_circle(center_x,center_y)
        for j in range(len(circle)):
            list_circular_point_list.append(circle[j])
        # call the create_a_circle function to the get the list of points on the circumference of the circle 
        # append the list to the list_circular_point_list 
    
    cloud_points =  list_circular_point_list #[[1.0, 1.0, 0.0],[1.0, 2.0, 0.0]]
    #pdb.set_trace()
    header = std_msgs.msg.Header() #header
    header.stamp = rospy.Time.now()
    #header.frame_id = 'bebopbase_footprint'
    header.frame_id = 'world'
    scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points) #create pcl from points
    pcl_pub.publish(scaled_polygon_pcl) #publish 
    
if __name__ == '__main__':
    try:
        rospy.init_node('pcl2_obs_pub')
        pcl_pub = rospy.Publisher("/obs1_pcl_topic", PointCloud2, queue_size=10) 
        r = rospy.Rate(10) # 10hz
        rospy.Subscriber("/obs_poses_list", Obsposelist, publish_pointcloud) #subscribe to the topic publishing list of obstacle's poses  (created in lab8_part1) .callback function for this topic publish_pointcloud
        rospy.spin()
    except rospy.ROSInterruptException:  pass
    