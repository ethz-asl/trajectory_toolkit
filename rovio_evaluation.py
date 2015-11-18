# Imports
import os, sys, inspect
from TimedData import TimedData
from Plotter import Plotter
import Quaternion
import Utils
import RosDataAcquisition
from RosDataAcquisition import RosbagStampedTopicLoader
import numpy as np

rovioOutputBag = '2015-11-17-14-56-45.bag'
rovioOutputTopic = '/rovio/odometry'
viconGroundtruthBag = '/home/michael/datasets/FlyingWithBurri/2015-11-16-10-12-40.bag'
viconGroundtruthTopic = '/bluebird/vrpn_client/estimated_transform'
startcut = 25
endcut = 40

plotterRon = Plotter(1, [1,1])
plotterAtt = Plotter(2, [4,1])
plotterPos = Plotter(3, [3,1])
plotterVel = Plotter(4, [3,1])
plotterRor = Plotter(5, [3,1])
plotterYpr = Plotter(6, [3,1])

td_rovio = TimedData(58)
rovio_posID = [1,2,3]
rovio_attID = [4,5,6,7]
rovio_velID = [8,9,10]
rovio_rorID = [11,12,13]
rovio_ronID = 14
rovio_posCovID = np.arange(15,24)
rovio_attCovID = np.arange(24,33)
rovio_posSpID = [33,34,36]
rovio_posSmID = [37,38,39]
rovio_yprID = [40,41,42]
rovio_yprCovID = np.arange(43,52)
rovio_yprSpID = [52,53,54]
rovio_yprSmID = [55,56,57]
td_vicon = TimedData(18)
vicon_posID = [1,2,3]
vicon_attID = [4,5,6,7]
vicon_velID = [8,9,10]
vicon_rorID = [11,12,13]
vicon_ronID = 14
vicon_yprID = [15,16,17]

RosDataAcquisition.rosBagLoadOdometry(rovioOutputBag, rovioOutputTopic ,td_rovio,rovio_posID[0],rovio_attID[0],rovio_velID[0],rovio_rorID[0],rovio_posCovID[0],rovio_attCovID[0])
td_rovio.cropTimes(td_rovio.getFirstTime()+startcut,td_rovio.getLastTime()-endcut)
td_rovio.applyTimeOffset(-td_rovio.getFirstTime())
td_rovio.computeNormOfColumns(rovio_rorID,rovio_ronID)

RosDataAcquisition.rosBagLoadTransformStamped(viconGroundtruthBag,viconGroundtruthTopic,td_vicon,vicon_posID[0],vicon_attID[0])
td_vicon.cropTimes(td_vicon.getFirstTime()+startcut,td_vicon.getLastTime()-endcut)
td_vicon.computeRotationalRateFromAttitude(vicon_attID[0],vicon_rorID[0],2,2)
td_vicon.computeNormOfColumns(vicon_rorID,vicon_ronID)
 
to = td_vicon.getTimeOffset(vicon_ronID,td_rovio,rovio_ronID)
td_vicon.applyTimeOffset(-to)

plotterRon.addDataToSubplot(td_rovio, rovio_ronID, 1, 'r', 'rovio rotational rate norm')
plotterRon.addDataToSubplot(td_vicon, vicon_ronID, 1, 'b', 'vicon rotational rate norm')

td_vicon.computeVelocitiesInBodyFrameFromPostionInWorldFrame(vicon_posID, vicon_velID, vicon_attID)

td_rovio.applyBodyTransform(rovio_posID[0], rovio_attID[0], np.zeros(3), np.array([0,1,0,0]))
td_rovio.applyBodyTransformToAttCov(rovio_attCovID, np.array([0,1,0,0]))
td_rovio.applyBodyTransformToTwist(rovio_velID[0], rovio_rorID[0], np.zeros(3), np.array([0,1,0,0]))
td_rovio.applyBodyTransform(rovio_posID[0], rovio_attID[0], np.zeros(3), np.array([(1-0.6*0.6)**(1./2),0,0.6,0]))
td_rovio.applyBodyTransformToAttCov(rovio_attCovID, np.array([(1-0.6*0.6)**(1./2),0,0.6,0]))
td_rovio.applyBodyTransformToTwist(rovio_velID[0], rovio_rorID[0], np.zeros(3), np.array([(1-0.6*0.6)**(1./2),0,0.6,0]))
td_rovio.applyInertialTransform(rovio_posID[0], rovio_attID[0], np.zeros(3), np.array([0,0,0,1]))

B_r_BC_est, qCB_est = td_vicon.calibrateBodyTransform(vicon_velID[0], vicon_rorID[0], td_rovio, rovio_velID[0],rovio_rorID[0])
print('Calibrate Body Transform:')
print('Quaternion Rotation qCB_est:\tw:' + str(qCB_est[0]) + '\tx:' + str(qCB_est[1]) + '\ty:' + str(qCB_est[2]) + '\tz:' + str(qCB_est[3]))
print('Translation Vector B_r_BC_est:\tx:' + str(B_r_BC_est[0]) + '\ty:' + str(B_r_BC_est[1]) + '\tz:' + str(B_r_BC_est[2]))
  
J_r_JI_est, qIJ_est = td_vicon.calibrateInertialTransform(vicon_posID[0], vicon_attID[0], td_rovio, rovio_posID[0],rovio_attID[0], B_r_BC_est, qCB_est, [0,1,2,3,4,5])
print('Calibrate Inertial Transform:')
print('uaternion Rotation qIJ_est:\tw:' + str(qIJ_est[0]) + '\tx:' + str(qIJ_est[1]) + '\ty:' + str(qIJ_est[2]) + '\tz:' + str(qIJ_est[3]))
print('Translation Vector J_r_JI_est:\tx:' + str(J_r_JI_est[0]) + '\ty:' + str(J_r_JI_est[1]) + '\tz:' + str(J_r_JI_est[2]))
  
td_vicon.applyBodyTransform(vicon_posID[0], vicon_attID[0], B_r_BC_est, qCB_est)
td_vicon.applyInertialTransform(vicon_posID[0], vicon_attID[0],J_r_JI_est,qIJ_est)
td_vicon.applyBodyTransformToTwist(vicon_velID[0], vicon_rorID[0], B_r_BC_est, qCB_est)

td_rovio.computeSigmaBounds(rovio_posID,[rovio_posCovID[0],rovio_posCovID[4],rovio_posCovID[8]],rovio_posSpID,rovio_posSmID,3)
plotterPos.addDataToSubplotMultiple(td_rovio, rovio_posID, [1,2,3], ['r','r','r'], ['','',''])
plotterPos.addDataToSubplotMultiple(td_rovio, rovio_posSmID, [1,2,3], ['r--','r--','r--'], ['','',''])
plotterPos.addDataToSubplotMultiple(td_rovio, rovio_posSpID, [1,2,3], ['r--','r--','r--'], ['','',''])
plotterPos.addDataToSubplotMultiple(td_vicon, vicon_posID, [1,2,3], ['b','b','b'], ['','',''])

plotterVel.addDataToSubplotMultiple(td_rovio, rovio_velID, [1,2,3], ['r','r','r'], ['','',''])
plotterVel.addDataToSubplotMultiple(td_vicon, vicon_velID, [1,2,3], ['b','b','b'], ['','',''])

plotterAtt.addDataToSubplotMultiple(td_rovio, rovio_attID, [1,2,3,4], ['r','r','r','r'], ['','','',''])
plotterAtt.addDataToSubplotMultiple(td_vicon, vicon_attID, [1,2,3,4], ['b','b','b','b'], ['','','',''])

plotterRor.addDataToSubplotMultiple(td_rovio, rovio_rorID, [1,2,3], ['r','r','r'], ['','',''])
plotterRor.addDataToSubplotMultiple(td_vicon, vicon_rorID, [1,2,3], ['b','b','b'], ['','',''])

td_rovio.quaternionToYpr(rovio_attID,rovio_yprID)
td_rovio.quaternionToYprCov(rovio_attID,rovio_attCovID,rovio_yprCovID)
td_rovio.computeSigmaBounds(rovio_yprID,[rovio_yprCovID[0],rovio_yprCovID[4],rovio_yprCovID[8]],rovio_yprSpID,rovio_yprSmID,3)
td_vicon.quaternionToYpr(vicon_attID,vicon_yprID)
plotterYpr.addDataToSubplotMultiple(td_rovio, rovio_yprID, [1,2,3], ['r','r','r'], ['','',''])
plotterYpr.addDataToSubplotMultiple(td_rovio, rovio_yprSmID, [1,2,3], ['r--','r--','r--'], ['','',''])
plotterYpr.addDataToSubplotMultiple(td_rovio, rovio_yprSpID, [1,2,3], ['r--','r--','r--'], ['','',''])
plotterYpr.addDataToSubplotMultiple(td_vicon, vicon_yprID, [1,2,3], ['b','b','b'], ['','',''])
 
raw_input("Press Enter to continue...")
