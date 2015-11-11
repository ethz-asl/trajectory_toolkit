# from pylab import *
# import numpy as np
# cmd_subfolder = "../python"
# if cmd_subfolder not in sys.path:
# 	sys.path.insert(0, cmd_subfolder)
# import listener
# listener = reload(listener)
# from listener import Plotter
# import matplotlib.animation as animation

# Set execution bools
hasRosSubscriber = False;
isLivePlotting = False;

# Imports (Execution specific)
import os, sys, inspect
import matplotlib.pyplot as plt
from TimedData import TimedData
if hasRosSubscriber:
	import rospy
	from RosDataAcquisition import TransformStampedListener
else:
	from RosDataAcquisition import RosbagStampedTopicLoader

# Create a TimeData Object
td1 = TimedData(11)

# Init node or load ros bag
if hasRosSubscriber:
	rospy.init_node('test', anonymous=True)
	rate = rospy.Rate(100)
	tsl = TransformStampedListener(td1,"/vicon/firefly_sbx/firefly_sbx",1,4)


# matplotlib.pyplot.close('all')
# plt.ion()
# matplotlib.rcParams['text.usetex'] = True
# matplotlib.rcParams['pdf.fonttype'] = 42
# matplotlib.rcParams['ps.fonttype'] = 42
# plotter1 = Plotter(1)
# plotter2 = Plotter(2)
# rospy.Subscriber("/vicon/firefly_sbx/firefly_sbx", TransformStamped, plotter2.callback)

# Acquire Data
if hasRosSubscriber:
	while not rospy.is_shutdown():
	 	if isLivePlotting:
	 		plotter1.refresh()
	 		plotter2.refresh()
	 	rate.sleep()
else:
	rbLoader = RosbagStampedTopicLoader('dataset.bag', '/vicon/firefly_sbx/firefly_sbx');
	rbLoader.loadTransformStamped(td1,1,4);

# Post-processing
# td1.computeVeloctiyFromPosition(1, 8);

# Plotting
plt.plot(td1.d[0:td1.last,0],td1.d[0:td1.last,1])
plt.plot(td1.d[0:td1.last,0],td1.d[0:td1.last,2])
plt.show()
