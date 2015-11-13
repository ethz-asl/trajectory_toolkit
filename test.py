# from pylab import *
# import numpy as np
# cmd_subfolder = "../python"
# if cmd_subfolder not in sys.path:
# 	sys.path.insert(0, cmd_subfolder)
# import listener
# listener = reload(listener)
# import matplotlib.animation as animation

# Set execution bools
hasRosSubscriber = False;
isLivePlotting = False;

# Imports (Execution specific)
import os, sys, inspect
import matplotlib.pyplot as plt
from TimedData import TimedData
from Plotter import Plotter
import numpy as np
import Quaternion


if hasRosSubscriber:
	import rospy
	from RosDataAcquisition import TransformStampedListener
else:
	from RosDataAcquisition import RosbagStampedTopicLoader


# Create a TimeData Object
td1 = TimedData(8)
td2 = TimedData(8)
td1.basicTests();
td1.advancedTests();
Quaternion.tests();
 
# Init node or load ros bag
if hasRosSubscriber:
	rospy.init_node('test', anonymous=True)
	rate = rospy.Rate(100)
	tsl = TransformStampedListener(td1,"/vicon/firefly_sbx/firefly_sbx",1,4)
 
plotter1 = Plotter(1, [3,1], isLivePlotting)
plotter1.addDataToSubplot(td1, 1, 1, 'g');
plotter1.addDataToSubplot(td2, 1, 1, 'b', 'Position Y');
 
# Acquire Data
if hasRosSubscriber:
	while not rospy.is_shutdown():
	 	if isLivePlotting:
	 		plotter1.refresh()
	 	rate.sleep()
else:
	rbLoader = RosbagStampedTopicLoader('2015-11-11-17-18-29.bag', '/rovio/transform');
	rbLoader.loadTransformStamped(td1,1,4);
	rbLoader = RosbagStampedTopicLoader('2015-11-11-17-23-26.bag', '/rovio/transform');
	rbLoader.loadTransformStamped(td2,1,4);
  
# Post-processing
# td1.computeVeloctiyFromPosition(1, 8);
  
# Plotting
plotter1.show();

