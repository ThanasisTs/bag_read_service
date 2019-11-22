import pandas as pd
from datetime import datetime
import csv
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import sys
import re
import string
from mpl_toolkits.mplot3d import Axes3D

filename = sys.argv[1]

df = pd.read_csv(filename)

# Function to extract all the distinct keypoints from a csv 
def getKeyPoints(columns): 
	columns = string.join(columns)
	array = re.findall(r"/keypoint_3d_matching/keypoints/[0-9]+/points/point/x", columns)
	return array 

# Function to extract all the distinct keypoint names from a csv 
def getKeypointNames(columns): 
	listOfNames = []
	for col in columns[1:]:
		if "name" in col:
			listOfNames.append(df[col].loc[0])
	return listOfNames

def get_cmap(n, name='hsv'):
    '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct 
    RGB color; the keyword argument name must be a standard mpl colormap name.'''
    return plt.cm.get_cmap(name, n)

listOfNames = getKeypointNames(df.columns)
numOfKeyPoints = len(getKeyPoints(df.columns))
cmap = get_cmap(numOfKeyPoints)

listOfKeyPoints_x = ["/keypoint_3d_matching/keypoints/" + str(i) + "/points/point/x" for i in range(numOfKeyPoints)]
listOfKeyPoints_y = ["/keypoint_3d_matching/keypoints/" + str(i) + "/points/point/y" for i in range(numOfKeyPoints)]
listOfKeyPoints_z = ["/keypoint_3d_matching/keypoints/" + str(i) + "/points/point/z" for i in range(numOfKeyPoints)]

new_df = pd.DataFrame()
for point_list in [listOfKeyPoints_x, listOfKeyPoints_y, listOfKeyPoints_z]:
	for col in point_list:
		new_df[col] = df[col]

fig = plt.figure()
ax = Axes3D(fig)

for i, (list_x, list_y, list_z) in enumerate(zip(listOfKeyPoints_x, listOfKeyPoints_y, listOfKeyPoints_z)):
	ax.scatter(df[list_x], df[list_y], df[list_z], color=cmap(i), label=listOfNames[i]) 

ax.set_xlabel('x-axis')
ax.set_ylabel('y-axis')
ax.set_zlabel('z-axis')
ax.legend()
plt.show()