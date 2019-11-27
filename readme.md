### Description

Open a __new terminal__ and run an openpose_utils launch file.
Example: 
        roslaunch openpose_utils_launch openpose_sole.launch

In a __new terminal__ run
        
        roslaunch bag_read_service bag_read_service.launch
The `bag_read_service.launch` starts the server `readBagService.py` with the name of the rosbag file as an argument. `readBagService.py` provides the service of reading the rosbag file and publishes its topics.

The client node `clientBag.py`, which is also initiated, takes the topic where the `keypoint_3d_matching` publishes as argument. Every time that there is a new message on this topic the client requests the service of `readBagService.py`.

In a __new terminal__ run 

        rosservice call /next_msg

to read the first message from the rosbag file and start the pipeline.
