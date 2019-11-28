import pandas as pd
import sys
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import rosbag
import numpy as np
import matplotlib.pyplot as plt

filename = sys.argv[1]
df = pd.read_csv(filename)

def get_cmap():
	return cycle('bgrcmk')	

size = len(df["Img_stamp"])

df["Img_value"] = np.ones(size)

df["PointCloud_value"] = np.ones(size)*1.1

ax = plt.gca()

df.plot(kind='scatter', x='PointCloud_stamp', y='Img_value', color='red', ax=ax, s=0.05)
df.plot(kind='scatter', x='Img_stamp', y='PointCloud_value', color='blue', ax=ax, s=0.05, ylim=[0, 3])

labels = ['PointCloud','Img']
ax.set_xlabel('timestamps')
ax.set_ylabel('msgs')
plt.legend(labels)
plt.show()
