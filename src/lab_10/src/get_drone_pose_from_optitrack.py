#!/usr/bin/env python3  
import rospy
import tf2_ros
#import gazebo_msgs.msg
import geometry_msgs.msg
#import time
#from  lab_10.msg import Obsposelist
#import pdb
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#import tf_conversions
from nav_msgs.msg import Odometry #TODO include odometry from nav_msgs.msg 

broadcaster = None
last_published = None
publish_frequency = None
odom_publisher = None

def get_odom_base_footprint(transform, data):
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_footprint"
    transform.transform.translation.x = data.pose.pose.position.x # TODO get x position from the data
    transform.transform.translation.y = data.pose.pose.position.y # TODO get y position from the data
    transform.transform.translation.z = 0
    #TODO Question why are we setting transform.transform.translation.z to 0
    # we are not interested in altidue w/ respect to obstacles
    # the drone's height may be considered a constant, thus translation along z may be disregarded
    #TODO get the drone's quaternion from the data.pose 
    #TODO covert the drone's quaternion to  the roll,pitch , yaw of drone 
    #above two task can be done in one line     
    (_, _, drone_yaw) = euler_from_quaternion((data.pose.pose.orientation.x, 
                                               data.pose.pose.orientation.y, 
                                               data.pose.pose.orientation.z, 
                                               data.pose.pose.orientation.w)) 
    #TODO  get drone quaternion from data and then use euler_from_quaternion function  to covert quat to euler
    
    #TODO set the roll and pitch to zero and convert the euler angles to quaternion 
    base_foot_print_quaternion = quaternion_from_euler(0, 0, drone_yaw)
    
    #TODO Question why we are setting roll and pitch value to zero and not yaw?
    # we only want the drone to turn left or right, rolling/pitching may cause instability
    # there's no need to roll/pitch the drone
    
    transform.transform.rotation.x = base_foot_print_quaternion[0]
    transform.transform.rotation.y = base_foot_print_quaternion[1] #TODO
    transform.transform.rotation.z = base_foot_print_quaternion[2] #TODO
    transform.transform.rotation.w = base_foot_print_quaternion[3] #TODO
   
    return transform

def pub_callback(data):
    global last_published, broadcaster, publish_frequency, odom_publisher
    #transforms =[]
    
    #get the bebop_orientation from data
    bebop_orientation = data.pose.orientation #TODO                                      
    
    #Following lines create an Odometry messages and publish it in odom_publisher
    bebop_odom = Odometry()
    bebop_odom.header.frame_id = 'odom'
    bebop_odom.child_frame_id = 'bebop_odom'
    bebop_odom.pose.pose.position = data.pose.position
    bebop_odom.pose.pose.orientation = bebop_orientation
    odom_publisher.publish(bebop_odom)
    
    #following three lines create a new tf frame connected odom to base_footprint
    transform = geometry_msgs.msg.TransformStamped()
    transform = get_odom_base_footprint(transform,bebop_odom)
    broadcaster.sendTransform(transform)

if __name__ == '__main__':
    rospy.init_node('drone_optitrack_pose_broadcaster')
    odom_publisher = rospy.Publisher("bebop_optitrack/odom", Odometry, queue_size = 1)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    publish_frequency = rospy.get_param("publish_frequency", 10)
    # TODO create a subsrciber which subscribe to /vrpn_client_node/Bebop/pose of type PoseStamped and use pub_callback function as callback
    #one line
    # change <Bebop#> depending on optitrack setting
    rospy.Subscriber("/vrpn_client_node/Bebop/pose", PoseStamped, pub_callback)
    rospy.spin()