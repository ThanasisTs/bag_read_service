#!/usr/bin/env python
import rosbag
import roslib
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from openpose_ros_msgs.msg import OpenPoseHumanList
from openpose_ros_receiver_msgs.msg import Keypoints_v
        
class listen3D(object):
    def __init__(self):
        self.published_count = 0
        self.published_count2 = 0

        self.sub = rospy.Subscriber("/openpose_ros_receiver/robot_frame_coords_msg", Keypoints_v, self.callback)
        self.sub2 = rospy.Subscriber("/openpose_ros/human_list", OpenPoseHumanList, self.callback2)
        self.rate = rospy.Rate(10)
        self.prev_keypoints = None
        self.distinct_msgs = 1

    def stripXpoint(self, keypoint):
        return round(keypoint.points.point.x, 3)

    def checkSimilarity(self, keypoints):
        for keypoint, prev_keypoint in zip(keypoints, self.prev_keypoints):
            if self.stripXpoint(prev_keypoint) != self.stripXpoint(keypoint):
                self.distinct_msgs += 1
                return False
        return True

    def callback(self, data):
        
        if self.prev_keypoints is not None:
           print(self.checkSimilarity(data.keypoints))
        
        self.prev_keypoints = data.keypoints
        self.published_count += 1

        #self.rate.sleep()

    def callback2(self, data):                     
            self.published_count2 += 1

            #self.rate.sleep()


def main():
    rospy.init_node('checkFrames', anonymous=True)
    lb = listen3D()
    
    while not rospy.is_shutdown():
        rospy.spin()

    print("Listener: Total published msgs: %d "%(lb.published_count))
    print("Listener: Distinct published msgs: %d "%(lb.distinct_msgs))


    print("Listener: Openpose_wrapper published msgs: %d "%(lb.published_count2))

    
if __name__ == '__main__':
    main()