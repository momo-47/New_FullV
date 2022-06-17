#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import time
import math

odom = []

def callback(msg):
  odom=msg.data
    
    
    
    
def odoms(od):

  odom_msg = Odometry()

  w1=od[0]
  w2=od[1]
  dist1=od[2]
  dist2=od[3]

  dist=(0.056/2.0)*(dist1+dist2)
  
  vx=(0.056/2.0)*(w1+w2)
  vy=0
  vz=0
  
  odom_msg.twist.twist.linear.x = vx
  odom_msg.twist.twist.linear.y = vy
  odom_msg.twist.twist.linear.z = vz

  wx=0
  wy=0
  wz=(0.056/0.4)*(w1-w2)

  odom_msg.twist.twist.angular.x = wx
  odom_msg.twist.twist.angular.y = wy
  odom_msg.twist.twist.angular.z = wz

  
  odom_msg.twist.covariance[0] = vx
  odom_msg.twist.covariance[1] = vy
  odom_msg.twist.covariance[2] = vz
  odom_msg.twist.covariance[3] = wx
  odom_msg.twist.covariance[4] = wy
  odom_msg.twist.covariance[5] = wz

  
  theeta=(0.056/0.4)*(dist1-dist2)
  y=(t*vx*math.sin(theeta))+y;
  x=(t*vx*math.cos(theeta))+x;
  z=0;

  odom_msg.pose.pose.position.x = x
  odom_msg.pose.pose.position.y = y
  odom_msg.pose.pose.position.z = z

  
  odom_msg.pose.pose.orientation = tf.transformations.quaternion_from_euler(0,0,theeta)
 


  odom_msg.pose.covariance[0] = x;
  odom_msg.pose.covariance[1] = y;
  odom_msg.pose.covariance[2] = z;
  odom_msg.pose.covariance[3] = 0;
  odom_msg.pose.covariance[4] = 0;
  odom_msg.pose.covariance[5] = theeta;
  
  odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
  )
  
  odom_msg.child_frame_id = "base_link"

  
  odom_msg.header.stamp = rospy.Time.now()
  odom_msg.header.frame_id = "odom"
  
  

  pub.publish(odom_msg)
        
    
def zmex():

    rospy.init_node('zmex', anonymous=True)

    rospy.Subscriber("noisy_odom", Float32MultiArray, callback)
    
    pub = rospy.Publisher('odom', Odometry, queue_size=50)
    odom_broaadcaster = tf.TransformBroadcaster()
    odoms(odom)


    rospy.spin()

if __name__ == '__main__':
    zmex()
