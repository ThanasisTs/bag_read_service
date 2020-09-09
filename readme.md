### Description

##### Rosbag Read (RR) Server

###### Service Provided
- Read from a rosbag file one message at a time for every topic
- Publish each message at its topic

##### Rosbag Read Client

###### Functionality
- Listen to a specific topic.
- When a message from this topic is received, ask a RR service from the server.

##### Server-Client Loop
- Call the service of the RR server once to start the loop
- The openpose_utils pipeline receives the messages from the rosbag file and eventually publishes a list of 3D keypoints.
- The client listens to this topic.
- The loop stops when there is a topic in the rosbag file with no other messages.



### Run Code

Open a __new terminal__ and run an [openpose_utils](https://github.com/Roboskel-Manipulation/openpose_utils) launch file.

Example: 

        roslaunch openpose_utils_launch openpose_sole.launch bag_file:="filename"

In a __new terminal__ run
        
        roslaunch bag_read_service bag_read_service.launch bag_file:="filename"
The `bag_read_service.launch` starts the server `readBagService.py`. In the `bag_read_service.launch` update the `bag_file` parameter with the name of the file you want to read.
`readBagService.py` provides the service of reading the rosbag file and publishes its topics.

The client node `clientBag.py`, which is also initiated, uses the listen_topic parameter to define which topic the client should listen at. Every time that there is a new message on this topic the client requests the service of `readBagService.py`.

In a __new terminal__ run 

        rosservice call /next_msg

to read the first message from the rosbag file and start the pipeline.
