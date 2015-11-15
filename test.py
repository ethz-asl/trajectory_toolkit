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
isLivePlotting = True;

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


plotter1 = Plotter(1, [3,1], isLivePlotting)
plotter2 = Plotter(2, [3,1], isLivePlotting)

# Create a TimeData Object
td1 = TimedData(3)
td2 = TimedData(3)
td1.initEmptyFromTimes(np.arange(0,10,0.01))
td2.initEmptyFromTimes(np.arange(0,10,0.01))
td1.setColumnToSine(1, 1, 0.1, 0)
td2.setColumnToSine(1, 1, 0.1, np.pi/4)
td1.setCol(np.abs(td1.col(1)), 2)
td2.setCol(np.abs(td2.col(1)), 2)
plotter1.addDataToSubplot(td1, 2, 1, 'g');
plotter1.addDataToSubplot(td2, 2, 1, 'b');
to = td1.getTimeOffset(td2, 2)
print(to)
td1.applyTimeOffset(-to)
plotter2.addDataToSubplot(td1, 2, 1, 'g');
plotter2.addDataToSubplot(td2, 2, 1, 'b');
# td1.basicTests();
# Quaternion.tests();
 
# # Init node or load ros bag
# if hasRosSubscriber:
# 	rospy.init_node('test', anonymous=True)
# 	rate = rospy.Rate(100)
# 	tsl = TransformStampedListener(td1,"/vicon/firefly_sbx/firefly_sbx",1,4)
#  
# plotter1 = Plotter(1, [1,1], isLivePlotting)
# plotter1.addDataToSubplot(td1, 1, 1, 'g');
# plotter1.addDataToSubplot(td2, 1, 1, 'b');
#  
# # Acquire Data
# if hasRosSubscriber:
# 	while not rospy.is_shutdown():
# 	 	if isLivePlotting:
# 	 		plotter1.refresh()
# 	 	rate.sleep()
# else:
# 	rbLoader = RosbagStampedTopicLoader('/home/michael/datasets/result61.bag', '/rovio/transform');
# 	rbLoader.loadTransformStamped(td1,1,4);
# 	rbLoader = RosbagStampedTopicLoader('/home/michael/datasets/result62.bag', '/rovio/transform');
# 	rbLoader.loadTransformStamped(td2,1,4);
#    
# # Post-processing
# # td1.computeVeloctiyFromPosition(1, 8);
#    
# # Plotting
# plotter1.show();

input("Press Enter to continue...")