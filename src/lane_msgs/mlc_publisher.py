#!/usr/bin/env python
import rospy
from lane_msgs.msg import Mlc

def mlc_publisher(mlc):
    pub = rospy.Publisher('mlc_data', Mlc, queue_size=10)
    rospy.init_node('mlc_publisher', anonymous=True)
    r = rospy.Rate(10) #10hz

    while not rospy.is_shutdown():
        rospy.loginfo(mlc)
        pub.publish(mlc)
        r.sleep()

if __name__ == '__main__':
    try:
        mlc = Mlc()
        mlc_publisher(mlc)
    except rospy.ROSInterruptException: pass
