#!/usr/bin/env python
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import roslib, rospy, rosbag
from std_srvs.srv import Empty,EmptyResponse, Trigger
import sys

class BagByService():

    def __init__(self, bagfile):
        self.bag = rosbag.Bag(bagfile, 'r')

        self.publishers = {}
        self.messages = {}
        self.listOfCounts = {}
        self.listOfEndConditions = {}
        self.msg_types = [CameraInfo, Image, Image, PointCloud2]

        #Create a publisher for each topic
        self.topics, _ = self.get_topic_and_type_list()
        for topic, msg_type in zip(self.topics, self.msg_types):

            # create list of publishers for each topic
            self.publishers[topic] = rospy.Publisher(topic, msg_type, queue_size = 10)

            # create a list of messages for each topic
            self.messages[topic] = self.bag.read_messages(topics= topic)

            # a counter of msgs for each topic
            self.listOfCounts[topic] = 0

            # flag when generator emptied for each topic
            self.listOfEndConditions[topic] = False

        # create a Server
        self.next_msgs_srv = rospy.Service('/next_msg', Empty, self.service)

    def get_topic_and_type_list(self):
        # retrieve a list of topics from the rosbag file
        topics = self.bag.get_type_and_topic_info()[1].keys()

        # retrieve a list of message types from the rosbag file
        # NOT WORKING PROPERLY | USE THE HARD-CODED LIST
        types=[]
        for i in range(0,len(self.bag.get_type_and_topic_info()[1].values())):
            types.append(self.bag.get_type_and_topic_info()[1].values()[i][0])

        result_topic = []  
        result_type = []  
        for to,ty in zip(topics,types):
            result_topic.append(to)
            result_type.append(ty)

        return result_topic, result_type 

    def pub_next_msg(self):
        for topic in self.topics:
            try:
                _, msg, _ = self.messages[topic].next()
            except:
                self.listOfEndConditions[topic] = True
                continue
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

    #run this file with the name of the bag file
    bbs = BagByService(sys.argv[1])

    while not rospy.is_shutdown():
        if all(value for value in bbs.listOfEndConditions.values()):
            break
        rospy.spin()

    for topic in bbs.topics:
        print("Bag Reader: Total published msgs from topic %s: %d "%(topic, bbs.listOfCounts[topic]))
    
    bbs.bag.close()