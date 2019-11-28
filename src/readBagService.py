#!/usr/bin/env python
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import roslib, rospy, rosbag
from std_srvs.srv import Empty,EmptyResponse, Trigger
import sys
from rosgraph_msgs.msg import Clock, Log

class BagByService():

    def __init__(self, bagfile):
        self.bag = rosbag.Bag(bagfile, 'r')

        self.publishers = {}
        self.messages = {}
        self.listOfCounts = {}
        self.listOfEndConditions = {}
        # self.msg_types = [CameraInfo, Image, Image, PointCloud2, Clock]

        #Create a publisher for each topic
        self.topics, self.msg_types = self.get_topic_and_type_list()

        for topic in self.topics:

            # create list of publishers for each topic
            self.publishers[topic] = rospy.Publisher(topic, self.msg_types[topic], queue_size = 10)

            # create a list of messages for each topic
            self.messages[topic] = self.bag.read_messages(topics= topic)

            # a counter of msgs for each topic
            self.listOfCounts[topic] = 0

            # flag when generator emptied for each topic
            self.listOfEndConditions[topic] = False
        # publisher for clock
        self.clockPublisher = rospy.Publisher("/clock", Clock, queue_size = 10)
        # create a Server
        self.next_msgs_srv = rospy.Service('/next_msg', Empty, self.service)

    def get_topic_and_type_list(self):
        # retrieve a list of topics from the rosbag file
        info = self.bag.get_type_and_topic_info()
        topics = list(info[1].keys())
        # retrieve a list of message types from the rosbag file
        # NOT WORKING PROPERLY | USE THE HARD-CODED LIST
        types = {}
        for topic in topics:
            types[topic] = eval((info[1][topic][0]).split("/")[1])
        # print types
        # input()
        return topics, types 

    def pub_next_msg(self):
        for topic in self.topics:
            try:
                _, msg, t = self.messages[topic].next()
            except:
                self.listOfEndConditions[topic] = True
                continue
            if topic == "/camera/rgb/image_raw":
                self.clockPublisher.publish(t)

            self.publishers[topic].publish(msg)
            self.listOfCounts[topic] += 1

    def service(self, req):
        try:
            self.pub_next_msg()
            return EmptyResponse()
        except RuntimeError,e:
            rospy.logerr("Exception caught:\n%s", e)
            return None 

if __name__=='__main__':

    rospy.init_node('bag_by_service')

    bag_file = rospy.get_param('~bag_file') 
    #bag_file = sys.argv[1]
    #run this file with the name of the bag file
    bbs = BagByService(bag_file)

    while not rospy.is_shutdown():
        if all(value for value in bbs.listOfEndConditions.values()):
            break
        rospy.spin()

    for topic in bbs.topics:
        print("Bag Reader: Total published msgs from topic %s: %d "%(topic, bbs.listOfCounts[topic]))
    
    bbs.bag.close()