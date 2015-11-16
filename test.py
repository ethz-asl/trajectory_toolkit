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
import Utils


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
td2.setColumnToSine(1, 1, frequency, 2 * np.pi * frequency * to)
td1.setCol(np.abs(td1.col(1)), 2)
td2.setCol(np.abs(td2.col(1)), 2)
plotter1.addDataToSubplot(td1, 2, 1, 'g');
plotter1.addDataToSubplot(td2, 2, 1, 'b');
to = td1.getTimeOffset(td2, 2)
td2.applyTimeOffset(to)
plotter1.addDataToSubplot(td1, 2, 2, 'g');
plotter1.addDataToSubplot(td2, 2, 2, 'b');

# Test alignement
td3 = TimedData(18)
td4 = TimedData(18)
rbLoader = RosbagStampedTopicLoader('2015-11-11-17-18-29.bag', '/rovio/transform');
rbLoader.loadTransformStamped(td3,1,4);
rbLoader = RosbagStampedTopicLoader('2015-11-11-17-18-29.bag', '/rovio/transform');
rbLoader.loadTransformStamped(td4,1,4);
td3.computeRotationalRateFromAttitude(4,8);
td4.computeRotationalRateFromAttitude(4,8);
td3.computeNormOfColumns([8,9,10],11);
td4.computeNormOfColumns([8,9,10],11);
td3.computeVectorNDerivative([1,2,3],[12,13,14]);
td4.computeVectorNDerivative([1,2,3],[12,13,14]);
td3.computeVelocitiesInBodyFrameFromPostionInWorldFrame([1,2,3],[15,16,17],td3.cols([4,5,6,7]));
td4.computeVelocitiesInBodyFrameFromPostionInWorldFrame([1,2,3],[15,16,17],td4.cols([4,5,6,7]));

# Time offset
to = 2.7;
td4.applyTimeOffset(-to)
plotter2.addDataToSubplot(td3, 11, 1, 'g');
plotter2.addDataToSubplot(td4, 11, 1, 'b');
to = td3.getTimeOffset(td4, 2)
td4.applyTimeOffset(to)
plotter2.addDataToSubplot(td3, 11, 2, 'g');
plotter2.addDataToSubplot(td4, 11, 2, 'b');

# Transform body frame
q = Quaternion.q_exp(np.array([0.1,0.2,0.32]))
v = np.array([1.1,-0.2,0.4])
td3.applyBodyTransform(1,4,v,q)

plotter2.addDataToSubplot(td3, 15, 3, 'r');
plotter2.addDataToSubplot(td3, 16, 3, 'g');
plotter2.addDataToSubplot(td3, 17, 3, 'b');
# td4.invertTransform(1,4)
# td4.applyInertialTransform(1,4,-Quaternion.q_rotate(q,v),Quaternion.q_inverse(q))
# td4.invertTransform(1,4)
plotter2.addDataToSubplot(td4, 15, 3, 'r');
plotter2.addDataToSubplot(td4, 16, 3, 'g');
plotter2.addDataToSubplot(td4, 17, 3, 'b');

td3.computeRotationalRateFromAttitude(4,8);
td4.computeRotationalRateFromAttitude(4,8);
td3.computeVelocitiesInBodyFrameFromPostionInWorldFrame([1,2,3],[15,16,17],td3.cols([4,5,6,7]));
td4.computeVelocitiesInBodyFrameFromPostionInWorldFrame([1,2,3],[15,16,17],td4.cols([4,5,6,7]));

translation, rotation = td4.calibrateBodyTransform(15,8,td3,15,8)
# translationI, rotationI = td4.calibrateInertialTransform(12,8,td3,12,8,[0,1,2,3,4], rotation, translation)
# print(translationI)
# print(rotationI)
print(Quaternion.q_log(rotation))
print(translation)
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