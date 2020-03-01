import pandas as pd
import matplotlib.pyplot as plt
import sys
import re
import string
import numpy as np
from itertools import cycle
from matplotlib.patches import Polygon
from scipy import stats

__author__ = "Lygerakis Fotios"
__license__ = "GPL"
__email__ = "ligerfotis@gmail.com"

title = "Index-Thumb Groundtruth"

#box plot aperture set
top = 6
bottom = 0	

# plot the average every frameAvg points
frameAvg = 7

# Ground truth points
# Marker Frame
A = [-34.60, 29.52, 0.0]
B = [-44.76, 21.90, 0.0]
C = [-29.52, 37.14, 0.0]
D = [-29.52, 24.44, 0.0]
I = [-38.09, 15.87, 0.0]
T1 = [-37.46, 16.50, 0.0]
T2 = [-36.19, 17.30, 0.0]
T3 = [-36.03, 18.09, 0.0]
T4 = [-35.40, 18.57, 0.0]
T5 = [-34.60, 19.36, 0.0]

# froundtruth printed on the scatter plot
gt_point_index = I 
gt_point_thumb = T5

# axes rangeD
axis_range_x = [-0.6, -0.28]
axis_range_y = [0.10, 0.40]
axis_range_z = [0.0, 0.15]

# steps for tick printing on the axes
step_x = .01
step_y = .01
step_z = .01

# used in boxplot | positioned under each box
aperture_size = ['1cm', '2cm', '3cm', '4cm', '5cm']
msgsPerFile = ["291", "288", "290", "289", "291"]
img_raw_msgs = ["178", "176", "206", "200"]
pc_msgs = ["180", "174", "204", "198"]

# the z score value used to filter outliers
zscore = 2.698

# Function to extract all the distinct keypoints from a csv 
def getKeyPoints(columns): 
	columns = "".join(columns)
	array = re.findall(r"/topic_transform/keypoints/[0-9]+/points/point/x", columns)
	return array 

# Function to extract all the distinct keypoint names from a csv 
def getKeypointNames(df): 
	listOfNames = []
	for col in df.columns[1:]:
		if "name" in col:
			listOfNames.append(df[col].loc[0])
	return listOfNames

def get_cmap():
    return cycle('brgcmk')


def keypointExtractor(filename, withOutliers):
	df = pd.read_csv(filename)
	
	listOfNames = getKeypointNames(df)
	numOfKeyPoints = len(getKeyPoints(df.columns))

	listOfKeyPoints_x = ["/topic_transform/keypoints/" + str(i) + "/points/point/x" for i in range(numOfKeyPoints)]
	listOfKeyPoints_y = ["/topic_transform/keypoints/" + str(i) + "/points/point/y" for i in range(numOfKeyPoints)]
	listOfKeyPoints_z = ["/topic_transform/keypoints/" + str(i) + "/points/point/z" for i in range(numOfKeyPoints)]

	new_listOfKeyPoints_x, new_listOfKeyPoints_y, new_listOfKeyPoints_z = [], [], []
	new_listOfNames = []
	# create new dataframe containing only columns with point information 
	new_df = pd.DataFrame()
	for i, point_list_names in enumerate(zip(listOfKeyPoints_x, listOfKeyPoints_y, listOfKeyPoints_z)):
		# check if keypoint contains mostly zero points
		count_valid_array = [(df[col_point]!=0).sum() for col_point in point_list_names]

		if all([point > numOfKeyPoints/4 for point in count_valid_array]):
			new_listOfKeyPoints_x.append(point_list_names[0])
			new_listOfKeyPoints_y.append(point_list_names[1])
			new_listOfKeyPoints_z.append(point_list_names[2])
			new_listOfNames.append(listOfNames[i])

			for col in point_list_names:
				# rounding is crucial for comparing
				new_df[col] = df[col].astype(float).round(7)

	# replace zero values with NAN so they do not plot
	new_df.replace(0, np.nan, inplace=True)

	# remove outliers
	if not withOutliers:
		new_df = new_df[(np.abs(stats.zscore(new_df)) < zscore).all(axis=1)]

	return new_df, listOfKeyPoints_x, listOfKeyPoints_y, listOfKeyPoints_z, new_listOfNames

def extractAperture(filename):
	df, new_listOfKeyPoints_x,new_listOfKeyPoints_y, new_listOfKeyPoints_z,_ = keypointExtractor(filename, withOutliers=True)

	# calculate aperture
	aperture = []
	point_list = []
	for keypoint_x, keypoint_y in zip(new_listOfKeyPoints_x, new_listOfKeyPoints_y):
		point_list.append([df[keypoint_x].values, df[keypoint_y].values])

	susbtraction_x = point_list[0][0] - point_list[1][0]
	substraction_y = point_list[0][1] - point_list[1][1]	

	# euclidean distance
	aperture = np.sqrt(np.add(np.power(susbtraction_x, 2), np.power(substraction_y,2)))
	aperture = aperture[~np.isnan(aperture)]

	return aperture

def boxPlot():

	filenames = sys.argv[2:]
	
	apertures = [extractAperture(filename) for filename in filenames]

	data_cm = [aperture * 100 for aperture in apertures] 

	labels = [ap+"\nTotal messages: "+msg for ap, msg in zip(aperture_size, msgsPerFile)]

	fig, ax = plt.subplots(figsize=(10, 6))
	fig.canvas.set_window_title('Aperture between Thumb and Index Fingertips in cm')
	

	medianprops = dict(linestyle=None, linewidth=0, color='white')
	bp = ax.boxplot(data_cm, notch=0, sym='+', vert=1, whis=1.5, showmeans=True, meanline=True, medianprops=medianprops)
	plt.setp(bp['boxes'], color='black')
	plt.setp(bp['whiskers'], color='black')
	plt.setp(bp['fliers'], color='red', marker='+')

	# Add a horizontal grid to the plot, but make it very light in color
	# so we can use it for reading data values but not be distracting
	ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
	               alpha=0.5)
	# Hide these grid behind plot objects
	ax.set_axisbelow(True)
	ax.set_title('Comparison of Aperture Discritability')
	ax.set_xlabel('Distripution')
	ax.set_ylabel('Aperture in cm')

	# Now fill the boxes with desired colors
	box_colors = ['darkkhaki', 'royalblue']
	num_boxes = len(data_cm)
	means = [np.mean(keypoint) for keypoint in data_cm]
	stds = [np.std(keypoint) for keypoint in data_cm]

	for i in range(num_boxes):
	    box = bp['boxes'][i]
	    boxX = []
	    boxY = []
	    for j in range(5):
	        boxX.append(box.get_xdata()[j])
	        boxY.append(box.get_ydata()[j])
	    box_coords = np.column_stack([boxX, boxY])
	    # Alternate between Dark Khaki and Royal Blue
	    ax.add_patch(Polygon(box_coords, facecolor=box_colors[i % 2]))
	# Set the axes ranges and axes labels
	ax.set_xlim(0.5, num_boxes + 0.5)

	plt.yticks(np.arange(bottom, top, 1))
	ax.set_xticklabels(labels, fontsize=8)

	plt.show()

def print2dPlot( axis1, axis2, new_df, new_listOfKeyPoints_x,new_listOfKeyPoints_y, new_listOfKeyPoints_z, new_listOfNames):

	fig, ax = plt.subplots(figsize=(10, 6))
	cmap = get_cmap()
	scat = None
	color = 'brgcmk'

	if ( [axis1, axis2] == ['x', 'y']):
		new_listOfKeyPoints_ax1 = new_listOfKeyPoints_x
		new_listOfKeyPoints_ax2 = new_listOfKeyPoints_y
	elif ( [axis1, axis2] == ['x', 'z']):
		new_listOfKeyPoints_ax1 = new_listOfKeyPoints_x
		new_listOfKeyPoints_ax2 = new_listOfKeyPoints_z

	elif ( [axis1, axis2] == ['y', 'z']):
		new_listOfKeyPoints_ax1 = new_listOfKeyPoints_y
		new_listOfKeyPoints_ax2 = new_listOfKeyPoints_z
	
	for i, (list_ax1, list_ax2) in enumerate(zip(new_listOfKeyPoints_ax1, new_listOfKeyPoints_ax2)):
		avglist1, avglist2 = getAvg(new_df[list_ax1], new_df[list_ax2], frameAvg)
		scat = plt.scatter(avglist1, avglist2, s=0.7, color=color[i], label=new_listOfNames[i]) 

	fig.suptitle(title)
	ax.set_xlabel( axis1 + '-axis (in m)')
	ax.set_ylabel( axis2 + '-axis (in m)')


	if ( [axis1, axis2] == ['x', 'y']):
		plt.xticks(np.arange(axis_range_x[0], axis_range_x[1], step=step_x))
		plt.yticks(np.arange(axis_range_y[0], axis_range_y[1], step=step_y))
		ax.set_xlim(axis_range_x)
		ax.set_ylim(axis_range_y)

		# get two gaussian random numbers, mean=0, std=1, 2 numbers
		gt_points_ax1 = [gt_point_index[0], gt_point_thumb[0]]
		gt_points_ax2 = [gt_point_index[1], gt_point_thumb[1]]

	elif ( [axis1, axis2] == ['x', 'z']):
		plt.xticks(np.arange(axis_range_x[0], axis_range_x[1], step=step_x))
		plt.yticks(np.arange(axis_range_z[0], axis_range_z[1], step=step_z))
		ax.set_xlim(axis_range_x)
		ax.set_ylim(axis_range_z)

		# get two gaussian random numbers, mean=0, std=1, 2 numbers
		gt_points_ax1 = [gt_point_index[0], gt_point_thumb[0]]
		gt_points_ax2 = [gt_point_index[2], gt_point_thumb[2]]

	elif ( [axis1, axis2] == ['y', 'z']):
		plt.xticks(np.arange(axis_range_y[0], axis_range_y[1], step=step_y))
		plt.yticks(np.arange(axis_range_z[0], axis_range_z[1], step=step_z))
		ax.set_xlim(axis_range_y)
		ax.set_ylim(axis_range_z)

		# get two gaussian random numbers, mean=0, std=1, 2 numbers
		gt_points_ax1 = [gt_point_index[1], gt_point_thumb[1]]
		gt_points_ax2 = [gt_point_index[2], gt_point_thumb[2]]
	

	mean_ax1 = np.mean(new_df[new_listOfKeyPoints_ax1])
	mean_ax2 = np.mean(new_df[new_listOfKeyPoints_ax2])
	std_ax1 = np.std(new_df[new_listOfKeyPoints_ax1])
	std_ax2 = np.std(new_df[new_listOfKeyPoints_ax2])


	plt.scatter(gt_points_ax1, gt_points_ax2, s=15, color=color[-1])

	ax.annotate("Ground Truth \nThumb Tip", (gt_points_ax1[0], gt_points_ax2[0]))
	ax.annotate("Ground Truth \nIndex Tip", (gt_points_ax1[1], gt_points_ax2[1]))

	plt.errorbar(mean_ax1, mean_ax2,  markersize=2,fmt='o', color='black', ecolor='black', ms=20,  mfc='white',label="Mean")

	ax.minorticks_on()
	ax.grid(which='major', linestyle='-', linewidth='0.5', color='black')
	ax.grid(which='minor', linestyle=':', linewidth='0.5', color='black')
	
	ax.legend(markerscale=5)
	plt.show()

'''
	get an average of the point over avgNum frames
'''
def getAvg( array1, array2, avgNum = 2):
	new_array1 = []
	new_array2 = []

	for i in range(0, len(array1)-avgNum):

		step = i + avgNum
		new_array1.append(np.mean( array1[i:step]) ) 
		new_array2.append(np.mean( array2[i:step]))

	return new_array1, new_array2

def plot():
	try:
		filename = sys.argv[2]
	except:
		print("Invalid file name given")
		sys.exit()

	new_df, new_listOfKeyPoints_x,new_listOfKeyPoints_y, new_listOfKeyPoints_z, new_listOfNames = keypointExtractor(filename, withOutliers=False)
	new_df.to_csv("~/tmp.csv")

	print2dPlot( 'x', 'y', new_df, new_listOfKeyPoints_x,new_listOfKeyPoints_y, new_listOfKeyPoints_z, new_listOfNames)
	print2dPlot( 'x', 'z', new_df, new_listOfKeyPoints_x,new_listOfKeyPoints_y, new_listOfKeyPoints_z, new_listOfNames)
	print2dPlot( 'y', 'z', new_df, new_listOfKeyPoints_x,new_listOfKeyPoints_y, new_listOfKeyPoints_z, new_listOfNames)

if __name__== "__main__":
	if sys.argv[1] == "boxplot":
		boxPlot()
	elif sys.argv[1] == "plotkeypoints":
		plot()
	else:
		print("Wrong keyword.")
		print("Select \"boxplot\" or \"plotkeypoints\".")