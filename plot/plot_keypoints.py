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
from matplotlib.patches import Polygon

# Function to extract all the distinct keypoints from a csv 
def getKeyPoints(columns): 
	columns = string.join(columns)
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
    return cycle('bgrcmk')

def keypointExtractor(filename):
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
		# print(all([point < numOfKeyPoints/4 for point in count_valid_array]))
		# input()

		if all([point > numOfKeyPoints/4 for point in count_valid_array]):
			new_listOfKeyPoints_x.append(point_list_names[0])
			new_listOfKeyPoints_y.append(point_list_names[1])
			new_listOfKeyPoints_z.append(point_list_names[2])
			new_listOfNames.append(listOfNames[i])

			for col in point_list_names:
				new_df[col] = df[col]
	# replace zero values with NAN so they do not plot
	new_df.replace(0, np.nan, inplace=True)

	return new_df, listOfKeyPoints_x, listOfKeyPoints_y, listOfKeyPoints_z, new_listOfNames

def extractAperture(filename):
	df, new_listOfKeyPoints_x,new_listOfKeyPoints_y, new_listOfKeyPoints_z,_ = keypointExtractor(filename)
	# calculate aperture
	aperture = []
	point_list = []
	for keypoint_x, keypoint_y in zip(new_listOfKeyPoints_x, new_listOfKeyPoints_y):
		point_list.append([df[keypoint_x].values, df[keypoint_y].values])


	aperture_x = np.absolute(point_list[0][0] - point_list[1][0])
	aperture_y = np.absolute(point_list[0][1] - point_list[1][1])
	return aperture_x, aperture_y
	# mean_x = np.mean(aperture_x)
	# std_x = np.std(aperture_x)
	# mean_y = np.mean(aperture_y)
	# std_y = np.std(aperture_y)

def boxPlot():
	path = '/home/liger/depth_checker_keypoints_tf/'
	filenames = ['aperture_5cm_keypoints_tf.csv', 'aperture_10cm_keypoints_tf.csv', 'aperture_15cm_keypoints_tf.csv']

	aperture_x_5, _ = extractAperture(path+filenames[0])
	aperture_x_10, _ = extractAperture(path+filenames[1])
	aperture_x_15, _ = extractAperture(path+filenames[2])
	
	data = [aperture_x_5, aperture_x_10, aperture_x_15]
	data_cm = [aperture_x_5*100, aperture_x_10*100, aperture_x_15*100]
	aperture_size = ['5cm', '10cm', '15cm']
	msgsPerFile = ["141", "99", "125"]
	img_raw_msgs = ["146", "101", "129"]
	pc_msgs = ["142", "100", "125"]

	labels = [ap+"\nTotal messages: "+msg for ap, msg in zip(aperture_size, msgsPerFile)]

	fig, ax1 = plt.subplots(figsize=(10, 6))
	fig.canvas.set_window_title('Aperture between Thumb and Index Fingertips in cm')
	

	medianprops = dict(linestyle=None, linewidth=0, color='white')
	bp = ax1.boxplot(data_cm, notch=0, sym='+', vert=1, whis=1.5, showmeans=True, meanline=True, medianprops=medianprops)
	plt.setp(bp['boxes'], color='black')
	plt.setp(bp['whiskers'], color='black')
	plt.setp(bp['fliers'], color='red', marker='+')

	# Add a horizontal grid to the plot, but make it very light in color
	# so we can use it for reading data values but not be distracting
	ax1.yaxis.grid(True, linestyle='-', which='major', color='lightgrey',
	               alpha=0.5)
	# Hide these grid behind plot objects
	ax1.set_axisbelow(True)
	ax1.set_title('Comparison of Aperture Discritability')
	ax1.set_xlabel('Distripution')
	ax1.set_ylabel('Aperture in cm')

	# Now fill the boxes with desired colors
	box_colors = ['darkkhaki', 'royalblue']
	num_boxes = len(data)
	means = [np.mean(keypoint) for keypoint in data]
	stds = [np.std(keypoint) for keypoint in data]

	for i in range(num_boxes):
	    box = bp['boxes'][i]
	    boxX = []
	    boxY = []
	    for j in range(5):
	        boxX.append(box.get_xdata()[j])
	        boxY.append(box.get_ydata()[j])
	    box_coords = np.column_stack([boxX, boxY])
	    # Alternate between Dark Khaki and Royal Blue
	    ax1.add_patch(Polygon(box_coords, facecolor=box_colors[i % 2]))
	# Set the axes ranges and axes labels
	ax1.set_xlim(0.5, num_boxes + 0.5)
	top = 15
	bottom = 0
	plt.yticks(np.arange(bottom, top, 1))
	# ax1.set_ylim(bottom, top)
	#plt.ticklabel_format(axis='y', style='sci', scilimits=(bottom,top))
	ax1.set_xticklabels(labels, fontsize=8)

	ax1.legend()
	plt.show()

def plot():
	try:
		filename = sys.argv[2]
	except:
		print("Invalid file name given")
		sys.exit()

	new_df, new_listOfKeyPoints_x,new_listOfKeyPoints_y, new_listOfKeyPoints_z, new_listOfNames = keypointExtractor(filename)

	# plot 3D scatter of points
	fig, ax = plt.subplots(figsize=(10, 6))
	cmap = get_cmap()
	for i, (list_x, list_y) in enumerate(zip(new_listOfKeyPoints_x, new_listOfKeyPoints_y)):
		plt.scatter(new_df[list_x], new_df[list_y], s=0.5, color=cmap.next(), label=new_listOfNames[i]) 

	ax.set_xlabel('x-axis')
	ax.set_ylabel('y-axis')


	axis_range_x = [-0.4, -0.2]
	axis_range_y = [-0.2, 0.2]

	plt.xticks(np.arange(axis_range_x[0], axis_range_x[1], step=0.05))
	plt.yticks(np.arange(axis_range_y[0], axis_range_y[1], step=0.1))

	ax.set_xlim(axis_range_x)
	ax.set_ylim(axis_range_y)

	mean_x = np.mean(new_df[new_listOfKeyPoints_x])
	mean_y = np.mean(new_df[new_listOfKeyPoints_y])
	std_x = np.std(new_df[new_listOfKeyPoints_x])
	std_y = np.std(new_df[new_listOfKeyPoints_y])

	plt.errorbar(mean_x, mean_y,  markersize='10',fmt='.', color='red', ecolor='black', ms=20, 
             elinewidth=4, capsize=10, capthick=4, label='Average')

	# textstr = '\n'.join((
	#     r'$\vec{\mu}_{thumb}=[%.2f, %.2f]$' % (mean_x[0], mean_y[0]),
	#     r'$\vec{\sigma}_{thumb}=[%.2f, %.2f]$' % (std_x[0], std_y[0])))

	# textstr = "Groundtruth Index = [-0.30, -0.00]"
	textstr = '\n'.join(("Groundtruth Thumb = [-0.30, 0.00]",
	    "Groundtruth Index = [-0.25, 0.00]"))
	# these are matplotlib.patch.Patch properties
	props = dict(facecolor='white', alpha=0.5)

	# place a text box in upper left in axes coords
	ax.text(0.008, 0.985, textstr, transform=ax.transAxes, fontsize=14,
	        verticalalignment='top', bbox=props)

	plt.grid()
	ax.legend()
	plt.show()

if __name__== "__main__":
	try:
		if sys.argv[1] == "boxplot":
			boxPlot()
		elif sys.argv[1] == "plotkeypoints":
			plot()
		else:
			print("Wrong keyword.")
			print("Select \"boxplot\" or \"plotkeypoints\".")
	except:
			print("Select \"boxplot\" or \"plotkeypoints\".")