#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, PointCloud2
import pandas as pd

listPointCloudTimestamps = []

total_msgs = 0
listImgTimestamps = []

def callbackPoint(data):
    listPointCloudTimestamps.append(data.header.stamp)


def callbackImg(data):
    total_msgs += 1
    listImgTimestamps.append(data.header.stamp)
  
def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, callbackPoint)

    rospy.Subscriber("/camera/rgb/image_raw", Image, callbackImg)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    print(total_msgs)
    #pd.DataFrame([listPointCloudTimestamps, listImgTimestamps], columns=['PointCloud', 'Img'])
    df_img = pd.DataFrame()
    df_img["Img_stamp"] = listImgTimestamps

    df_pc = pd.DataFrame()
    df_pc["PointCloud_stamp"] = listPointCloudTimestamps

    df = pd.concat([df_img,df_pc], ignore_index=False, axis=1)
    df.to_csv ("rosbag_info.csv", index = None, header=True)