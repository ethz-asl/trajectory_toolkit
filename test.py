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
td1.initEmptyFromTimes(np.arange(0,100,0.01))
td2.initEmptyFromTimes(np.arange(0,97,0.01))
td1.setColumnToSine(1, 1, 0.1, 0)
to = 0.2;
frequency = 0.1;
print(to)
td2.setColumnToSine(1, 1, frequency, 2 * np.pi * frequency * to)
td1.setCol(np.abs(td1.col(1)), 2)
td2.setCol(np.abs(td2.col(1)), 2)
plotter1.addDataToSubplot(td1, 2, 1, 'g');
plotter1.addDataToSubplot(td2, 2, 1, 'b');
to = td1.getTimeOffset(td2, 2)
print(to)
td2.applyTimeOffset(to)
plotter1.addDataToSubplot(td1, 2, 2, 'g');
plotter1.addDataToSubplot(td2, 2, 2, 'b');
# td1.basicTests();
# td1.advancedTests();
# Quaternion.tests();


td3 = TimedData(15)
td4 = TimedData(15)
rbLoader = RosbagStampedTopicLoader('/home/michael/catkin_ws/result31.bag', '/rovio/transform');
rbLoader.loadTransformStamped(td3,1,4);
rbLoader = RosbagStampedTopicLoader('/home/michael/catkin_ws/result31.bag', '/rovio/transform');
rbLoader.loadTransformStamped(td4,1,4);
td3.computeRotationalRateFromAttitude(4,8);
td4.computeRotationalRateFromAttitude(4,8);
td3.computeNormOfColumns([8,9,10],11);
td4.computeNormOfColumns([8,9,10],11);
td3.computeVectorNDerivative(1,12,3);
td4.computeVectorNDerivative(1,12,3);
to = 2.7;
print(to)
td4.applyTimeOffset(-to)
plotter2.addDataToSubplot(td3, 11, 1, 'g');
plotter2.addDataToSubplot(td4, 11, 1, 'b');
to = td3.getTimeOffset(td4, 2)
print(to)
td4.applyTimeOffset(to)
plotter2.addDataToSubplot(td3, 11, 2, 'g');
plotter2.addDataToSubplot(td4, 11, 2, 'b');



q = Quaternion.q_exp(np.array([0.1,0.2,0.32]))
print(q)
v = np.array([1.1,-0.2,0.4])
td3.applyBodyTransform(1,4,v,q)
plotter2.addDataToSubplot(td3, 1, 3, 'r');
plotter2.addDataToSubplot(td3, 2, 3, 'g');
plotter2.addDataToSubplot(td3, 3, 3, 'b');
# td4.invertTransform(1,4)
# td4.applyInertialTransform(1,4,-Quaternion.q_rotate(q,v),Quaternion.q_inverse(q))
# td4.invertTransform(1,4)
plotter2.addDataToSubplot(td4, 1, 3, 'r');
plotter2.addDataToSubplot(td4, 2, 3, 'g');
plotter2.addDataToSubplot(td4, 3, 3, 'b');

td3.computeRotationalRateFromAttitude(4,8);
td4.computeRotationalRateFromAttitude(4,8);
translation, rotation = td4.calibrateBodyTransform(12,8,td3,12,8)
print(rotation)
 
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

raw_input("Press Enter to continue...")