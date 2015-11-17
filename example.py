# Imports
import os, sys, inspect
import matplotlib.pyplot as plt
from TimedData import TimedData
from Plotter import Plotter
import numpy as np
import Quaternion
import Utils
from termcolor import colored
outputColor = 'blue'

"""
 Set isNode to true to run as a node here.
"""
isNode = False;

if isNode is not True:
	from RosDataAcquisition import RosbagStampedTopicLoader
	""" Initialize Plotters 
	plotter1: figureID=1, three subplots
	"""
	plotter1 = Plotter(1, [3,1])

	
	""" Initialize TimedData 
		td1: TimedData with 18 columns: 0:t, 1-3:pos, 4-7:att, 8-10:vel,  11-13:velInBodyFrame, 14-16:ror, 17:rorNorm
		td2: TimedData with 18 columns: 0:t, 1-4:att, 5-7:ror, 8:rorNorm, 9-11:pos, 12-14:vel, 15-17:velInBodyFrame
	"""
	td1 = TimedData(18)
	td2 = TimedData(18)
	posIDs1 = [1,2,3]
	posIDs2 = [9,10,11]
	attIDs1 = [4,5,6,7]
	attIDs2 = [1,2,3,4]
	velIDs1 = [8,9,10]
	velIDs2 = [12,13,14]
	velBIDs1 = [11,12,13]
	velBIDs2 = [15,16,17]
	rorIDs1 = [14,15,16]
	rorIDs2 = [5,6,7]
	rorNID1 = 17
	rorNID2 = 8
	
	"""
		First we will fill the two TimedData structures with the same StampedTransforms,
		loaded from topic 'rovio/transform' in example.bag.
		The indices denote the start column of the position(td1=1, td2=9) and attitude(td1=4, td2=1)
	"""
	rbLoader = RosbagStampedTopicLoader('example.bag', '/rovio/transform');
	rbLoader.loadTransformStamped(td1,posIDs1[0],attIDs1[0])
	rbLoader = RosbagStampedTopicLoader('example.bag', '/rovio/transform');
	rbLoader.loadTransformStamped(td2,posIDs2[0],attIDs2[0])
	
	# Add initial x to plot
	plotter1.addDataToSubplot(td1, posIDs1[0], 1, 'r', 'td1In x');
	plotter1.addDataToSubplot(td2, posIDs2[0], 1, 'b', 'td2In x');

	
	"""
		Apply body transform to the second data set. The column start IDs of position(9) and attitutde(1) have to be provided.
		We define the rotation through a rotation vector vCB, the exponential represents the corresponding rotation quaternion qCB.
		The translation is defined by the translation vector B_r_BC
	"""
	vCB = np.array([0.1,0.2,0.32])
	qCB = Quaternion.q_exp(vCB)
	B_r_BC = np.array([1.1,-0.2,0.4])
	td2.applyBodyTransform(posIDs2[0], attIDs2[0], B_r_BC, qCB)
	print(colored('Applying Body Transform:', outputColor))
	print('Rotation Vector ln(qCB):\tvx:' + str(vCB[0]) + '\tvy:' + str(vCB[1]) + '\tvz:' + str(vCB[2]))
	print('Translation Vector B_r_BC:\trx:' + str(B_r_BC[0]) + '\try:' + str(B_r_BC[1]) + '\trz:' + str(B_r_BC[2]))
	
	"""
		Apply inertial transform to the second data set. Same procedure as with the body transform.
	"""
	vIJ = np.array([0.2,-0.2,-0.4])
	qIJ = Quaternion.q_exp(vIJ)
	J_r_JI = np.array([-0.1,0.5,0.1])
	td2.applyInertialTransform(posIDs2[0], attIDs2[0],J_r_JI,qIJ)
	print(colored('Applying Inertial Transform:', outputColor))
	print('Rotation Vector ln(qIJ):\tvx:' + str(vIJ[0]) + '\tvy:' + str(vIJ[1]) + '\tvz:' + str(vIJ[2]))
	print('Translation Vector J_r_JI:\trx:' + str(J_r_JI[0]) + '\try:' + str(J_r_JI[1]) + '\trz:' + str(J_r_JI[2]))
	
	"""
		Apply time delay to the second data set.
	"""
	timeOffset = 0.2;
	td2.applyTimeOffset(timeOffset)
	print(colored('Applying Time Offset:', outputColor))
	print('Time Offset: ' + str(timeOffset) + 's')
	
	# Add transformed x to plot
	plotter1.addDataToSubplot(td1, posIDs1[0], 2, 'r', 'td1In x');
	plotter1.addDataToSubplot(td2, posIDs2[0], 2, 'b', 'td2Trans x');
	
	"""
		Now we are ready to calculate the other TimedData properties.
	"""
	# Calculate the velocity in the world frame provide pos(td1=[1,2,3], td2=[9,10,11]) and vel(td1=[8,9,10], td2=[12,13,14]) column IDs.
	td1.computeVectorNDerivative(posIDs1, velIDs1)
	td2.computeVectorNDerivative(posIDs2, velIDs2)
	# Calculate the velocity in the body frame provide pos(td1=[1,2,3], td2=[9,10,11]) and velInBodyFrame(td1=[11,12,13], td2=[15,16,17]) column IDs.
	# Additionally the rotation Quaternion qBI rps qCJ has to be provided.
	td1.computeVelocitiesInBodyFrameFromPostionInWorldFrame(posIDs1, velBIDs1, attIDs1)
	td2.computeVelocitiesInBodyFrameFromPostionInWorldFrame(posIDs2, velBIDs2, attIDs2)
	# Calculate the Rotational Rate provide att(td1=4, td2=1) and rot(td1=14, td2=5) start column IDs.
	td1.computeRotationalRateFromAttitude(attIDs1[0],rorIDs1[0])
	td2.computeRotationalRateFromAttitude(attIDs2[0],rorIDs2[0])
	# Calculate the Norm of the Rotational Rate provide ror(td1=[14,15,16], td2=[5,6,7]) and rorNorm(td1=17,td2=8) column IDs.
	td1.computeNormOfColumns(rorIDs1,rorNID1)
	td2.computeNormOfColumns(rorIDs2,rorNID2)
	
	"""
		We can estimate the time offset using the norm of the rotational rate.
		The estimated time offset is then applied to td2.
	"""
	to = td2.getTimeOffset(rorNID2,td1,rorNID1)
	td2.applyTimeOffset(-to)
	
	"""
		The calibration of the Body Transform needs the velocity and the rotational rate start IDs.
	"""
	B_r_BC_est, qCB_est = td1.calibrateBodyTransform(velBIDs1[0],rorIDs1[0],td2, velBIDs2[0],rorIDs2[0])
	vCB_est = Quaternion.q_log(qCB_est)
	vCB_err = vCB-vCB_est
	B_r_BC_err = B_r_BC - B_r_BC_est
	print(colored('Calibrate Body Transform:', outputColor))
	print('Rotation Vector ln(qCB_est):\tvx:' + str(vCB_est[0]) + '\tvy:' + str(vCB_est[1]) + '\tvz:' + str(vCB_est[2]))
	print('Translation Vector B_r_BC_est:\trx:' + str(B_r_BC_est[0]) + '\try:' + str(B_r_BC_est[1]) + '\trz:' + str(B_r_BC_est[2]))
	print('Rotation Error ln(qCB_err):\tvx:' + str(vCB_err[0]) + '\tvy:' + str(vCB_err[1]) + '\tvz:' + str(vCB_err[2]))
	print('Translation Error B_r_BC_err:\trx:' + str(B_r_BC_err[0]) + '\try:' + str(B_r_BC_err[1]) + '\trz:' + str(B_r_BC_err[2]))
	
	"""
		The calibration of the Intertial Transform needs the velocity and the rotational rate start IDs and the estimated body transform.
	"""
	J_r_JI_est, qIJ_est = td1.calibrateInertialTransform(posIDs1[0], attIDs1[0], td2, posIDs2[0], attIDs2[0], B_r_BC_est, qCB_est)
	vIJ_est = Quaternion.q_log(qIJ_est);
	vIJ_err = vIJ-vIJ_est;
	J_r_JI_err = J_r_JI - J_r_JI_est;
	print(colored('Calibrate Inertial Transform:', outputColor))
	print('Rotation Vector ln(qIJ_est):\tvx:' + str(vIJ_est[0]) + '\tvy:' + str(vIJ_est[1]) + '\tvz:' + str(vIJ_est[2]))
	print('Translation Vector J_r_JI_est:\trx:' + str(J_r_JI_est[0]) + '\try:' + str(J_r_JI_est[1]) + '\trz:' + str(J_r_JI_est[2]))
	print('Rotation Error ln(qIJ_err):\tvx:' + str(vIJ_err[0]) + '\tvy:' + str(vIJ_err[1]) + '\tvz:' + str(vIJ_err[2]))
	print('Translation Error J_r_JI_err:\trx:' + str(J_r_JI_err[0]) + '\try:' + str(J_r_JI_err[1]) + '\trz:' + str(J_r_JI_err[2]))
	
	
	# Add calibrated x to plot
	td1.applyBodyTransform(posIDs1[0], attIDs1[0], B_r_BC_est, qCB_est)
	td1.applyInertialTransform(posIDs1[0], attIDs1[0],J_r_JI_est,qIJ_est)

	plotter1.addDataToSubplot(td1, posIDs1[0], 3, 'r', 'td1Cal x');
	plotter1.addDataToSubplot(td2, posIDs2[0], 3, 'b', 'td2Trans x');
	plotter1.show()

else:
	import rospy
	from RosDataAcquisition import TransformStampedListener
	"""
		The above illustrated the functionality by loading data from a bagfile.
		This part (which can be enabled by setting isNode to True at the head of this file), shows the
		live plotting function of this tool. The trajectory_toolkit operates as a ros node and plots
		the data published on the given topic.
		To test this run 'roscore' in a terminal, and in a second run 'rosbag play example.bag'.
		Then execute 'python example.py' in a third terminal to run this node.
	"""
	"""
		Initialize the ros node and the transformStamped listener. Provide the column ID's where pos and att shall be stored.
	"""
	td3 = TimedData(8)
	rospy.init_node('example', anonymous=True)
	rate = rospy.Rate(100)
	tsl = TransformStampedListener(td3, "/rovio/transform", 1, 4)
	
	"""
		Initialize the plotter as a live plot. Limit the maximal number of displayed points to 300.
	"""
	livePlotter = Plotter(1, [4,2], True, 300)
	livePlotter.addDataToSubplot(td3, 1, 1, 'r', 'rx');
	livePlotter.addDataToSubplot(td3, 2, 3, 'g', 'ry');
	livePlotter.addDataToSubplot(td3, 3, 5, 'b', 'rz');
	livePlotter.addDataToSubplot(td3, 4, 2, 'r', 'qw');
	livePlotter.addDataToSubplot(td3, 5, 4, 'g', 'qx');
	livePlotter.addDataToSubplot(td3, 6, 6, 'b', 'qy');
	livePlotter.addDataToSubplot(td3, 7, 8, 'y', 'qz');

	"""
		Run the node and refresh the live Plotter.
	"""
	while not rospy.is_shutdown():
	 	livePlotter.refresh()
	 	rate.sleep()

	"""
		Show plot at the end.
	"""
	livePlotter.show();

raw_input("Press Enter to continue...")