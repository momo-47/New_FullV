#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    AX = (int(data.data[data.data.index("A")+1:data.data.index("B")]))
    AY = (int(data.data[data.data.index("B")+1:data.data.index("C")]))
    AZ = (int(data.data[data.data.index("C")+1:data.data.index("D")]))
    GX = (int(data.data[data.data.index("D")+1:data.data.index("E")]))
    GY = (int(data.data[data.data.index("E")+1:data.data.index("F")]))
    GZ = (int(data.data[data.data.index("F")+1:data.data.index("G")]))    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("imu", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
