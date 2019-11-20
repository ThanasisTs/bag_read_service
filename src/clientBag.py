#!/usr/bin/env python
import sys
import rospy
from std_srvs.srv import Empty, Trigger, EmptyRequest
from openpose_ros_receiver_msgs.msg import Keypoints_v

def callback(Keypoints_v):
	rospy.wait_for_service('/next_msg')
	try:
		# Create an empty request
		req = EmptyRequest()

		# use the service
		service(req)

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

if __name__ == "__main__":
	rospy.init_node('clientBag', anonymous=True)

	# Create the connection to the service.
	service = rospy.ServiceProxy('/next_msg', Empty)

	#run this file with the name of the topic
	topic_sub = rospy.Subscriber(sys.argv[1], Keypoints_v, callback)
	rospy.spin()
