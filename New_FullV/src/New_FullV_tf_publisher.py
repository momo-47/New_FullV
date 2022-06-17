#!/usr/bin/env python  
import roslib
import rospy
import tf
import time  
import math
import time
from nav_msgs.msg import Odometry

#create a quaternion
rotation_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)

#translation vector
translation_vector = (0.0, 0.0, 0.0)

def tf_callback (odom_msg):

  rotation_quaternion = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
  translation_vector = (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z)
  


if __name__ == '__main__':
	#init the node
    rospy.init_node('frame_a_frame_b_broadcaster_node')
    rospy.Subscriber ("/odom", Odometry, tf_callback)
    time.sleep(2)
    #create a transformation broadcaster (publisher)
    transform_broadcaster = tf.TransformBroadcaster()

   # while(not rospy.is_shutdown()):

        
        #time
    current_time = rospy.Time.now()

    transform_broadcaster.sendTransform(
            translation_vector, 
            rotation_quaternion,
            current_time, 
            "/base_link", "/odom") #child frame, parent frame
    time.sleep(0.5)

    rospy.spin()

    
