<<<<<<< HEAD
### Description
=======
# Plot 3d Keypoints
>>>>>>> refs/remotes/origin/master

Open a __new terminal__ and run an openpose_utils launch file.
Example: 
        roslaunch openpose_utils_launch openpose_sole.launch

Start the server `readBagService.py` with the name of the rosbag file as an argument from `bag_read_service` package that provides the service of  reading the rosbag file and publishes its topics in a __new terminal__.

        rosrun bag_read_service readBagService.py rosbag_file.bag

Start the client node `clientBag.py` in a __new terminal__. This node takes the topic where the `keypoint_3d_matching` publishes as argument. Every time that there is a new message on this topic the client requests the service of `readBagService.py.
        
        rosrun bag_read_service clientBag.py /keypoint_3d_matching

In a __new terminal__ run 

        rosservice call /next_msg

<<<<<<< HEAD
to read the first message from the rosbag file and start the pipeline.
=======
![Plot](keypoints.png)
>>>>>>> refs/remotes/origin/master
