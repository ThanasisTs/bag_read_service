#!/usr/bin/env python
import rosbag
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from openpose_ros_msgs.msg import OpenPoseHumanList
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from openpose_ros_receiver_msgs.msg import Keypoints_v

class readBagClass():

    def __init__(self, bag_file):
        # Open rosbag.
        self.bag = rosbag.Bag(bag_file, "r")
        self.listOfTopics = ["/camera/rgb/image_raw", "/camera/depth/image", "/camera/rgb/camera_info", "/camera/depth_registered/points"]
        self.listOfMessages = [self.bag.read_messages(topics= topic) for topic in self.listOfTopics]
        self.msg_type_list = [Image, Image, CameraInfo, PointCloud2]

        self.listOfEndConditions = [False, False, False, False]
        self.listOfCounts = [0, 0, 0, 0]

        self.publishersList = [rospy.Publisher(topic, msg_type, queue_size = 10) for topic, msg_type in zip( self.listOfTopics, self.msg_type_list)]
        
        listen_topic = rospy.get_param('~listen_topic') 
        self.topic_sub = rospy.Subscriber(listen_topic, Keypoints_v, self.callback)


    # every time that human_list publishes a msg, the callback funtion 
    # will be called and publish a new frame from the rosbaf file
    def callback(self, data):
        rospy.sleep(0.5)

        for i in range(len(self.listOfMessages)):
            try:
                _, msg, _ = self.listOfMessages[i].next()

                self.publishersList[i].publish(msg)
                self.listOfCounts[i] += 1

            except:
                self.listOfEndConditions[i] = True

        #rospy.sleep(0.5)
        

def main(args):
    rospy.init_node('readBagFile', anonymous=True)
    bag_file = rospy.get_param('~bag_file') 
    rb = readBagClass(bag_file)
    #rospy.sleep(0.5)
    rb.callback(None)

    while not rospy.is_shutdown():
        if all(rb.listOfEndConditions):
            break
        rospy.spin()

    rb.bag.close()


    for i in range(len(rb.listOfCounts)):
        print("Bag Reader: Total published msgs from topic %s: %d "%(rb.listOfTopics[i], rb.listOfCounts[i]))
    

if __name__ == '__main__':
    main(sys.argv)