#!/usr/bin/env python
import rosbag
import roslib
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

        
class listenBag(object):
    def __init__(self):
        self.published_count = 0
        self.sub =rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher('/open_simulation', Image, queue_size = 10)

    def callback(self, data):
        
        self.published_count += 1
        self.pub.publish(data)
        self.rate.sleep()


def main():
    rospy.init_node('listenBag', anonymous=True)
    lb = listenBag()
    
    while not rospy.is_shutdown():
        rospy.spin()

    print("Listener: Total published msgs: %d "%(lb.published_count))

    
if __name__ == '__main__':
    main()