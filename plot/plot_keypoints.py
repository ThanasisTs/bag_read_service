import pandas as pd
from datetime import datetime
import csv
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import sys
import re
import string
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from itertools import cycle

filename = sys.argv[1]

df = pd.read_csv(filename)
# Function to extract all the distinct keypoints from a csv 
def getKeyPoints(columns): 
	columns = string.join(columns)
	array = re.findall(r"/topic_transform/keypoints/[0-9]+/points/point/x", columns)
	return array 

# Function to extract all the distinct keypoint names from a csv 
def getKeypointNames(columns): 
	listOfNames = []
	for col in columns[1:]:
		if "name" in col:
			listOfNames.append(df[col].loc[0])
	return listOfNames

def get_cmap():
    return cycle('bgrcmk')

listOfNames = getKeypointNames(df.columns)
numOfKeyPoints = len(getKeyPoints(df.columns))
cmap = get_cmap()

listOfKeyPoints_x = ["/topic_transform/keypoints/" + str(i) + "/points/point/x" for i in range(numOfKeyPoints)]
listOfKeyPoints_y = ["/topic_transform/keypoints/" + str(i) + "/points/point/y" for i in range(numOfKeyPoints)]
listOfKeyPoints_z = ["/topic_transform/keypoints/" + str(i) + "/points/point/z" for i in range(numOfKeyPoints)]

new_listOfKeyPoints_x = []
new_listOfKeyPoints_y = []
new_listOfKeyPoints_z = []
new_listOfNames = []
# create new dataframe containing only columns with point information 
new_df = pd.DataFrame()
for i, point_list in enumerate(zip(listOfKeyPoints_x, listOfKeyPoints_y, listOfKeyPoints_z)):
	# check if keypoint contains mostly zero points
	count_valid_array = [(df[col_point]!=0).sum() for col_point in point_list]
	# print(all([point < numOfKeyPoints/4 for point in count_valid_array]))
	# input()

	if all([point > numOfKeyPoints/4 for point in count_valid_array]):
		new_listOfKeyPoints_x.append(point_list[0])
		new_listOfKeyPoints_y.append(point_list[1])
		new_listOfKeyPoints_z.append(point_list[2])
		new_listOfNames.append(listOfNames[i])

		for col in point_list:
			new_df[col] = df[col]
# replace zero values with NAN so they do not plot
new_df.replace(0, np.nan, inplace=True)

# plot 3D scatter of points
fig = plt.figure()
ax = Axes3D(fig)

for i, (list_x, list_y, list_z) in enumerate(zip(new_listOfKeyPoints_x, new_listOfKeyPoints_y, new_listOfKeyPoints_z)):
	ax.scatter(new_df[list_x], new_df[list_y], new_df[list_z], color=cmap.next(), label=new_listOfNames[i]) 

ax.set_xlabel('x-axis')
ax.set_ylabel('y-axis')
ax.set_zlabel('z-axis')
ax.legend()
plt.show()