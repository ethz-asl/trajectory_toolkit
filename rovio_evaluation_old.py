# Imports
import os, sys, inspect
from TimedData import TimedData
from Plotter import Plotter
import Quaternion
import Utils
import RosDataAcquisition
import CsvDataAcquisition
import numpy as np
from pylab import *
import matplotlib.pyplot as plt
import rosbag as rb
import struct
import pickle

# Default value
rovioOutputBag = ''
rovioOutputTopic = ''
rovioPclTopic = ''
rovioExtrinsicTopic = ''
okvisOutputFile = ''
okvisOutputTopic = ''
viconGroundtruthFile = ''
viconGroundtruthTopic = ''
startcut = 0
endcut = 0
MrMV = np.array([0.0, 0.0, 0.0])
qVM = np.array([1.0, 0, 0, 0])
bodyTransformForBetterPlotRangePos = np.zeros(3)
bodyTransformForBetterPlotRangeAtt = np.array([1.0, 0, 0, 0])
bodyAlignViconToRovio = False
bodyAlignOkvisToVicon = False
doRovio = True
doOkvis = False
plotRon = False
plotAtt = False
plotPos = True
plotVel = True
plotRor = True
plotYpr = True
plotFea = False
plotExt = False
plotLeutiDistances = []
leutiSpacing = 0

# File names and settings
ID = 2

if ID == 0: # MH_01_easy with OKVIS
    rovioOutputBag = '/home/michael/datasets/euroc/MH_01_easy/rovio/2016-02-05-11-39-05_25_4_6_2_0.bag'
    rovioOutputTopic = '/rovio/odometry'
    okvisOutputFile = '/home/michael/datasets/euroc/MH_01_easy/okvis/okvis_new_mono_paramest_MH_01_easy.bag'
    okvisOutputTopic = '/okvis/okvis_node/okvis_odometry'
    viconGroundtruthFile = '/home/michael/datasets/euroc/MH_01_easy/data.csv'
    startcut = 40
    endcut = 10
    bodyTransformForBetterPlotRangePos = np.zeros(3)
    bodyTransformForBetterPlotRangeAtt = Quaternion.q_mult(np.array([(1-0.6*0.6)**(1./2),0,0.6,0]),np.array([0.0,1.0,0.0,0.0]))
    doRovio = True
    doOkvis = False
    plotLeutiDistances = np.arange(1,21)
    leutiSpacing = 0

if ID == 1: # MH_05_difficult
    rovioOutputBag = '/home/michael/datasets/euroc/MH_05_difficult/rovio/2016-02-04-16-37-03_25_4_6_1_0.bag'
    rovioOutputTopic = '/rovio/odometry'
    rovioExtrinsicTopic = '/rovio/extrinsics0'
    viconGroundtruthFile = '/home/michael/datasets/euroc/MH_05_difficult/data.csv'
    startcut = 0
    endcut = 0
    bodyTransformForBetterPlotRangePos = np.zeros(3)
    bodyTransformForBetterPlotRangeAtt = Quaternion.q_mult(np.array([(1-0.6*0.6)**(1./2),0,0.6,0]),np.array([0.0,1.0,0.0,0.0]))
    doRovio = True
    doOkvis = False
    plotExt = True
    plotLeutiDistances = np.arange(1,21)
    leutiSpacing = 0

if ID == 2: # V1_03_difficult with OKVIS
    rovioOutputBag = '/home/michael/datasets/euroc/V1_01_easy/rovio/2016-02-04-16-45-52_25_4_6_1_0.bag'
#     rovioOutputBag = '/home/michael/datasets/euroc/V1_02_medium/rovio/2016-02-04-16-51-26_25_4_6_1_0.bag'
#     rovioOutputBag = '/home/michael/datasets/euroc/V1_03_difficult/rovio/2016-02-04-17-06-43_25_4_6_1_0.bag'
#     rovioOutputBag = '/home/michael/datasets/euroc/V2_01_easy/rovio/2016-02-04-17-10-32_25_4_6_1_0.bag'
#     rovioOutputBag = '/home/michael/datasets/euroc/V2_02_medium/rovio/2016-02-04-17-11-58_25_4_6_1_0.bag'
#     rovioOutputBag = '/home/michael/datasets/euroc/V2_03_difficult/rovio/2016-02-04-17-13-15_25_4_6_1_0.bag'
    rovioOutputTopic = '/rovio/odometry'
    rovioExtrinsicTopic = '/rovio/extrinsics0'
    okvisOutputFile = '/home/michael/datasets/euroc/V1_03_difficult/okvis/okvis_paramest_V1_03_difficult.bag'
    okvisOutputTopic = '/okvis/okvis_node/okvis_odometry'
    viconGroundtruthFile = '/home/michael/datasets/euroc/V1_01_easy/data.csv'
    startcut = 0
    endcut = 0
    bodyTransformForBetterPlotRangePos = np.zeros(3)
    bodyTransformForBetterPlotRangeAtt = Quaternion.q_mult(np.array([(1-0.6*0.6)**(1./2),0,0.6,0]),np.array([0.0,1.0,0.0,0.0]))
    MtMV = -3.80322e-02
    doRovio = True
    doOkvis = False
    plotExt = True
    plotLeutiDistances = np.arange(1,21)
    leutiSpacing = 0

if ID == 3: # FlyingWithBurri/rounds_6
    rovioOutputBag = '/home/michael/datasets/FlyingWithBurri/rounds_6/2016-01-14-11-27-37.bag'
    rovioOutputTopic = '/rovio/odometry'
    viconGroundtruthFile = '/home/michael/datasets/FlyingWithBurri/rounds_6/2015-11-16-10-12-40.bag'
    viconGroundtruthTopic = '/bluebird/vrpn_client/estimated_transform'
    startcut = 20
    endcut = 45
    MrMV = np.array([6.90120e-02, -2.78077e-02, -1.23948e-01]) # TODO: check
    qVM = np.array([1.42930e-03, 8.17427e-01, -1.17036e-02, 5.75911e-01])
    bodyTransformForBetterPlotRangePos = np.zeros(3)
    bodyTransformForBetterPlotRangeAtt = Quaternion.q_mult(np.array([(1-0.6*0.6)**(1./2),0,0.6,0]),np.array([0.0,1.0,0.0,0.0]))
    bodyAlignViconToRovio = True
    bodyAlignOkvisToVicon = False
    doRovio = True
    doOkvis = False
    plotLeutiDistances = np.arange(1,21)
    leutiSpacing = 0

if ID == 4: # bloesch_leo/planar_1
    rovioOutputBag = '/home/michael/datasets/bloesch_leo/planar_1/rovio/2016-02-02-17-18-09_25_4_6_2_0.bag'
    rovioOutputTopic = '/rovio/odometry'
    rovioPclTopic = '/rovio/pcl'
    viconGroundtruthFile = '/home/michael/datasets/bloesch_leo/planar_1/2015-12-22-14-15-42.bag'
    viconGroundtruthTopic = '/bluebird/vrpn_client/estimated_transform'
    startcut = 0
    endcut = 0
    MrMV = np.array([6.90120e-02, -2.78077e-02, -1.23948e-01]) # TODO: check
    qVM = np.array([1.42930e-03, 8.17427e-01, -1.17036e-02, 5.75911e-01])
    bodyTransformForBetterPlotRangePos = np.zeros(3)
    bodyTransformForBetterPlotRangeAtt = np.array([1.0,0.0,0.0,0.0])
    bodyAlignViconToRovio = True
    bodyAlignOkvisToVicon = True
    doRovio = True
    doOkvis = False
    plotFea = True
    plotLeutiDistances = [3]
    leutiSpacing = 5

if ID == 5: # Long Dataset
    rovioOutputBag = '/home/michael/datasets/long_trajectory_leutiLynen/rovio/2016-02-04-17-24-15_25_4_6_1_0.bag'
    rovioOutputTopic = '/rovio/odometry'
    okvisOutputFile = '/home/michael/datasets/long_trajectory_leutiLynen/okvis/okvis_paramest_long_trajectory.bag'
    okvisOutputTopic = '/okvis/okvis_node/okvis_odometry'
    viconGroundtruthFile = '/home/michael/datasets/long_trajectory_leutiLynen/long_trajectory.bag'
    viconGroundtruthTopic = '/vicon/sensor_cross/sensor_cross'
    startcut = 1 # account for okvis jump
    endcut = 0
    bodyAlignViconToRovio = True
    bodyAlignOkvisToVicon = True
    doRovio = True
    doOkvis = False
    plotLeutiDistances = [10,40,90,160,250,360]
    leutiSpacing = 0

if ID == 6: # V1_02_medium
    rovioOutputBag = '/home/michael/datasets/euroc/V1_02_medium/rovio/2016-01-21-13-51-33.bag'
    rovioOutputTopic = '/rovio/odometry'
    okvisOutputFile = '/home/michael/datasets/euroc/V1_02_medium/okvis/2015-12-22-16-11-13.bag'
    okvisOutputTopic = '/okvis/okvis_node/okvis_transform'
    viconGroundtruthFile = '/home/michael/datasets/euroc/V1_02_medium/data.csv'
    startcut = 0
    endcut = 0
    bodyTransformForBetterPlotRangePos = np.zeros(3)
    bodyTransformForBetterPlotRangeAtt = Quaternion.q_mult(np.array([(1-0.6*0.6)**(1./2),0,0.6,0]),np.array([0.0,1.0,0.0,0.0]))
    doRovio = True
    doOkvis = False
    plotLeutiDistances = np.arange(1,21)
    leutiSpacing = 0

if ID == 7: # Outliers
    rovioOutputBag = '/home/michael/datasets/bags_for_bloesch/rovio/2016-01-24-18-38-23.bag'
    rovioOutputTopic = '/rovio/odometry'
    okvisOutputFile = '/home/michael/datasets/euroc/V1_02_medium/okvis/2015-12-22-16-11-13.bag'
    okvisOutputTopic = '/okvis/okvis_node/okvis_transform'
    viconGroundtruthFile = '/home/michael/datasets/bags_for_bloesch/2015-12-11-15-06-05.bag'
    viconGroundtruthTopic = '/auk/vrpn_client/estimated_transform'
    startcut = 0
    endcut = 0
    bodyTransformForBetterPlotRangePos = np.zeros(3)
    bodyTransformForBetterPlotRangeAtt = Quaternion.q_mult(np.array([(1-0.6*0.6)**(1./2),0,0.6,0]),np.array([0.0,1.0,0.0,0.0]))
    bodyAlignViconToRovio = True
    bodyAlignOkvisToVicon = False
    doRovio = True
    doOkvis = False
    plotLeutiDistances = []
    leutiSpacing = 0

if ID == 8: # ultra fast
    rovioOutputBag = '/home/michael/datasets/bag_iros_2015_mic/ultra_fast/rovio/2016-01-25-14-25-32.bag'
    rovioOutputTopic = '/rovio/odometry'
    okvisOutputFile = '/home/michael/datasets/bag_iros_2015_mic/ultra_fast/okvis/okvis_paramest_ultra_fast_2.bag'
    okvisOutputTopic = '/okvis/okvis_node/okvis_odometry'
    viconGroundtruthFile = '/home/michael/datasets/bag_iros_2015_mic/ultra_fast/ultra_fast_2.bag'
    viconGroundtruthTopic = '/vicon/auk/auk'
    startcut = 0
    endcut = 0
    bodyTransformForBetterPlotRangePos = np.zeros(3)
    bodyTransformForBetterPlotRangeAtt = Quaternion.q_mult(np.array([(1-0.6*0.6)**(1./2),0,0.6,0]),np.array([0.0,1.0,0.0,0.0]))
    bodyAlignViconToRovio = True
    bodyAlignOkvisToVicon = True
    doRovio = True
    doOkvis = True
    plotLeutiDistances = np.arange(1,21)
    leutiSpacing = 0

if True: # Plotter initialization
    if plotRon:
        plotterRon = Plotter(-1, [1,1],'Norm of Rotational Rate',['time [s]'],['Rotational Rate Norm [rad/s]'],10000)
    if plotAtt:
        plotterAtt = Plotter(-1, [4,1],'Attitude Quaternion',['','','','time[s]'],['w[1]','x[1]','y[1]','z[1]'],10000)
    if plotPos:
        plotterPos = Plotter(-1, [3,1],'Position',['','','time[s]'],['x[m]','y[m]','z[m]'],10000)
    if plotVel:
        plotterVel = Plotter(-1, [3,1],'Robocentric Velocity',['','','time[s]'],['$v_x$[m/s]','$v_y$[m/s]','$v_z$[m/s]'],10000)
    if plotRor:
        plotterRor = Plotter(-1, [3,1],'Rotational Rate',['','','time[s]'],['$\omega_x$[rad/s]','$\omega_y$[rad/s]','$\omega_z$[rad/s]'],10000)
    if plotYpr:
        plotterYpr = Plotter(-1, [3,1],'Yaw-Pitch-Roll Decomposition',['','','time[s]'],['roll[rad]','pitch[rad]','yaw[rad]'],10000)
    if plotExt:
        plotterExt = Plotter(-1, [3,1],'Extrinsics Translational Part',['','','time[s]'],['x[m]','y[m]','z[m]'],10000)

if doRovio: # Rovio Timed Data
    td_rovio = TimedData()
    nFeatures = 25
    td_rovio.addLabelingIncremental('pos',3)
    td_rovio.addLabelingIncremental('att',4)
    td_rovio.addLabelingIncremental('vel',3)
    td_rovio.addLabelingIncremental('ror',3)
    td_rovio.addLabelingIncremental('ron',1)
    td_rovio.addLabelingIncremental('posCov',9)
    td_rovio.addLabelingIncremental('attCov',9)
    td_rovio.addLabelingIncremental('posSp',3)
    td_rovio.addLabelingIncremental('posSm',3)
    td_rovio.addLabelingIncremental('ypr',3)
    td_rovio.addLabelingIncremental('yprCov',9)
    td_rovio.addLabelingIncremental('yprSp',3)
    td_rovio.addLabelingIncremental('yprSm',3)
    td_rovio.addLabelingIncremental('extPos',3)
    td_rovio.addLabelingIncremental('extPosCov',9)
    td_rovio.addLabelingIncremental('extPosSp',3)
    td_rovio.addLabelingIncremental('extPosSm',3)
    td_rovio.addLabelingIncremental('extAtt',4)
    td_rovio.addLabelingIncremental('extAttCov',9)
    td_rovio.addLabelingIncremental('feaPos',3,nFeatures)
    td_rovio.addLabelingIncremental('feaCov',9,nFeatures)
    td_rovio.addLabelingIncremental('feaIdx',1,nFeatures)
    td_rovio.reInit()
 
if True: # Vicon Timed Data
    td_vicon = TimedData()
    td_vicon.addLabelingIncremental('pos',3)
    td_vicon.addLabelingIncremental('att',4)
    td_vicon.addLabelingIncremental('vel',3)
    td_vicon.addLabelingIncremental('ror',3)
    td_vicon.addLabelingIncremental('ron',1)
    td_vicon.addLabelingIncremental('ypr',3)
    td_vicon.reInit()
 
if doOkvis: # Okvis Timed Data
    td_okvis = TimedData()
    td_okvis.addLabelingIncremental('pos',3)
    td_okvis.addLabelingIncremental('att',4)
    td_okvis.addLabelingIncremental('vel',3)
    td_okvis.addLabelingIncremental('ror',3)
    td_okvis.addLabelingIncremental('ron',1)
    td_okvis.addLabelingIncremental('ypr',3)
    td_okvis.reInit()
 
if True: # Vicon data acquisition and pre-processing
    if(viconGroundtruthFile.endswith('.csv')):
        CsvDataAcquisition.csvLoadTransform(viconGroundtruthFile, 0, 1, 4, td_vicon, 'pos', 'att')
    elif(viconGroundtruthFile.endswith('.bag')):
        RosDataAcquisition.rosBagLoadTransformStamped(viconGroundtruthFile,viconGroundtruthTopic,td_vicon,'pos','att')
    if(viconGroundtruthFile.endswith('data.csv')):
        td_vicon.d[:,0] = td_vicon.d[:,0]*1e-9
    td_vicon.cropTimes(td_vicon.getFirstTime()+startcut,td_vicon.getLastTime()-endcut)
    td_vicon.applyTimeOffset(-td_vicon.getFirstTime())
    td_vicon.computeRotationalRateFromAttitude('att','ror',3,3)
    td_vicon.computeNormOfColumns('ror','ron')
    td_vicon.computeVelocitiesInBodyFrameFromPostionInWorldFrame('pos', 'vel', 'att',3,3)
 
if doRovio: # Rovio data acquisition and pre-processing
    RosDataAcquisition.rosBagLoadOdometry(rovioOutputBag, rovioOutputTopic ,td_rovio,'pos','att','vel','ror','posCov','attCov')
    if rovioPclTopic != '':
        RosDataAcquisition.rosBagLoadRobocentricPointCloud(rovioOutputBag,rovioPclTopic,td_rovio,'feaIdx','feaPos','feaCov')
    if rovioExtrinsicTopic != '':
        RosDataAcquisition.rosBagLoadPoseWithCovariance(rovioOutputBag, rovioExtrinsicTopic ,td_rovio,'extPos','extAtt','extPosCov','extAttCov')
    td_rovio.computeNormOfColumns('ror','ron')
    td_rovio.applyTimeOffset(td_vicon.getFirstTime()-td_rovio.getFirstTime())
    to = td_rovio.getTimeOffset('ron',td_vicon,'ron')
    td_rovio.applyTimeOffset(-to)
    td_rovio.cropTimes(td_vicon.getFirstTime(),td_vicon.getLastTime())
 
if doOkvis: # Okvis data acquisition and pre-processing
    if(okvisOutputFile.endswith('.csv')):
        CsvDataAcquisition.csvLoadTransform(okvisOutputFile, 0, 1, 4, td_okvis, 'pos', 'att')
    elif(okvisOutputFile.endswith('.bag')):
        RosDataAcquisition.rosBagLoadOdometry(okvisOutputFile,okvisOutputTopic,td_okvis,'pos','att','vel','ror')
    if(okvisOutputFile.endswith('data.csv')):
        td_okvis.d[:,0] = td_okvis.d[:,0]*1e-9
    if(okvisOutputFile.endswith('0.5speed.bag')):
        td_okvis.d[:,0] = 0.5*td_okvis.d[:,0]
    if(okvisOutputFile == '/home/michael/datasets/long_trajectory_leutiLynen/aslam/aslam_estimator_output_data.csv'):
        q_real = td_okvis.col('att'[3])
        q_im = -td_okvis.col('att'[0:3])
        td_okvis.setCol(q_real,'att'[0])
        td_okvis.setCol(q_im,'att'[1:4])
#     td_okvis.computeRotationalRateFromAttitude('att','ror',10,10)
    td_okvis.setCol(Quaternion.q_rotate(td_okvis.col('att'), td_okvis.col('ror')),'ror')
    td_okvis.computeNormOfColumns('ror','ron')
    td_okvis.applyTimeOffset(td_vicon.getFirstTime()-td_okvis.getFirstTime())
    to = td_okvis.getTimeOffset('ron',td_vicon,'ron')
    td_okvis.applyTimeOffset(-to)
    td_okvis.cropTimes(td_vicon.getFirstTime(),td_vicon.getLastTime())
    td_okvis.setCol(Quaternion.q_rotate(td_okvis.col('att'), td_okvis.col('vel')),'vel')
#     td_okvis.computeVelocitiesInBodyFrameFromPostionInWorldFrame('pos', 'vel', 'att',30,30)
          
if plotRon: # Plotting rotational rate norm
    if doRovio:
        plotterRon.addDataToSubplot(td_rovio, 'ron', 1, 'r', 'rovio rotational rate norm')
    plotterRon.addDataToSubplot(td_vicon, 'ron', 1, 'b', 'vicon rotational rate norm')
    if doOkvis:
        plotterRon.addDataToSubplot(td_okvis, 'ron', 1, 'g', 'okvis rotational rate norm')
  
if True: # Transform body coordinate frame for better vizualization, TODO: add to config
    if doRovio:
        td_rovio.applyBodyTransform('pos', 'att', bodyTransformForBetterPlotRangePos, bodyTransformForBetterPlotRangeAtt)
        td_rovio.applyBodyTransformToAttCov('attCov', bodyTransformForBetterPlotRangeAtt)
        td_rovio.applyBodyTransformToTwist('vel', 'ror', bodyTransformForBetterPlotRangePos, bodyTransformForBetterPlotRangeAtt)
        td_rovio.applyInertialTransform('pos', 'att', np.zeros(3), np.array([1.0,0,0,0]))
    if doOkvis:
        td_okvis.applyBodyTransform('pos', 'att', bodyTransformForBetterPlotRangePos, bodyTransformForBetterPlotRangeAtt)
        td_okvis.applyBodyTransformToTwist('vel', 'ror', bodyTransformForBetterPlotRangePos, bodyTransformForBetterPlotRangeAtt)
        td_okvis.applyInertialTransform('pos', 'att', np.zeros(3), np.array([1.0,0,0,0]))
          
if True: # Align Vicon to Rovio using calibration (body frame)
    B_r_BC_est = -Quaternion.q_rotate(qVM, MrMV) + Quaternion.q_rotate(Quaternion.q_inverse(qVM),bodyTransformForBetterPlotRangePos)
    qCB_est = Quaternion.q_mult(bodyTransformForBetterPlotRangeAtt,Quaternion.q_inverse(qVM))
    td_vicon.applyBodyTransform('pos', 'att', B_r_BC_est, qCB_est)
    td_vicon.applyBodyTransformToTwist('vel', 'ror', B_r_BC_est, qCB_est)
   
if doRovio and bodyAlignViconToRovio: # Align Vicon to Rovio (body frame)
    B_r_BC_est, qCB_est = td_vicon.calibrateBodyTransform('vel', 'ror', td_rovio, 'vel','ror')
    print('Calibrate Body Transform for Rovio:')
    print('Quaternion Rotation qCB_est:\tw:' + str(qCB_est[0]) + '\tx:' + str(qCB_est[1]) + '\ty:' + str(qCB_est[2]) + '\tz:' + str(qCB_est[3]))
    print('Translation Vector B_r_BC_est:\tx:' + str(B_r_BC_est[0]) + '\ty:' + str(B_r_BC_est[1]) + '\tz:' + str(B_r_BC_est[2]))
    td_vicon.applyBodyTransform('pos', 'att', B_r_BC_est, qCB_est)
    td_vicon.applyBodyTransformToTwist('vel', 'ror', B_r_BC_est, qCB_est)
  
if doOkvis and bodyAlignOkvisToVicon: # Align Okvis to Vicon (body frame)
    B_r_BC_est, qCB_est = td_okvis.calibrateBodyTransform('vel', 'ror', td_vicon, 'vel','ror')
    print('Calibrate Body Transform for Okvis:')
    print('Quaternion Rotation qCB_est:\tw:' + str(qCB_est[0]) + '\tx:' + str(qCB_est[1]) + '\ty:' + str(qCB_est[2]) + '\tz:' + str(qCB_est[3]))
    print('Translation Vector B_r_BC_est:\tx:' + str(B_r_BC_est[0]) + '\ty:' + str(B_r_BC_est[1]) + '\tz:' + str(B_r_BC_est[2]))
    td_okvis.applyBodyTransform('pos', 'att', B_r_BC_est, qCB_est)
    td_okvis.applyBodyTransformToTwist('vel', 'ror', B_r_BC_est, qCB_est)
      
if doRovio: # Align Vicon to Rovio (Inertial frames)
    J_r_JI_est, qIJ_est = td_vicon.calibrateInertialTransform('pos', 'att', td_rovio, 'pos','att', np.array([0.0,0.0,0.0]), np.array([1.0,0.0,0.0,0.0]), [0,1,2,3,4,5])
    print('Calibrate Inertial Transform for Rovio:')
    print('uaternion Rotation qIJ_est:\tw:' + str(qIJ_est[0]) + '\tx:' + str(qIJ_est[1]) + '\ty:' + str(qIJ_est[2]) + '\tz:' + str(qIJ_est[3]))
    print('Translation Vector J_r_JI_est:\tx:' + str(J_r_JI_est[0]) + '\ty:' + str(J_r_JI_est[1]) + '\tz:' + str(J_r_JI_est[2]))
    td_vicon.applyInertialTransform('pos', 'att',J_r_JI_est,qIJ_est)
  
if doOkvis: # Align Okvis to Vicon (Inertial frames)
    J_r_JI_est, qIJ_est = td_okvis.calibrateInertialTransform('pos', 'att', td_vicon, 'pos','att', np.array([0.0,0.0,0.0]), np.array([1.0,0.0,0.0,0.0]), [0,1,2,3,4,5,6])
    print('Calibrate Inertial Transform for Okvis:')
    print('uaternion Rotation qIJ_est:\tw:' + str(qIJ_est[0]) + '\tx:' + str(qIJ_est[1]) + '\ty:' + str(qIJ_est[2]) + '\tz:' + str(qIJ_est[3]))
    print('Translation Vector J_r_JI_est:\tx:' + str(J_r_JI_est[0]) + '\ty:' + str(J_r_JI_est[1]) + '\tz:' + str(J_r_JI_est[2]))
    td_okvis.applyInertialTransform('pos', 'att',J_r_JI_est,qIJ_est)
  
if plotPos: # Position plotting
    if doRovio:
        td_rovio.computeSigmaBounds('pos','posCov','posSp','posSm',3)
        plotterPos.addDataToSubplotMultiple(td_rovio, 'pos', [1,2,3], ['r','r','r'], ['','',''])
        plotterPos.addDataToSubplotMultiple(td_rovio, 'posSm', [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterPos.addDataToSubplotMultiple(td_rovio, 'posSp', [1,2,3], ['r--','r--','r--'], ['','',''])
    plotterPos.addDataToSubplotMultiple(td_vicon, 'pos', [1,2,3], ['b','b','b'], ['','',''])
    if doOkvis:
        plotterPos.addDataToSubplotMultiple(td_okvis, 'pos', [1,2,3], ['g','g','g'], ['','',''])
   
if plotVel: # Velocity plotting
    if doRovio:
        plotterVel.addDataToSubplotMultiple(td_rovio, 'vel', [1,2,3], ['r','r','r'], ['','',''])
    plotterVel.addDataToSubplotMultiple(td_vicon, 'vel', [1,2,3], ['b','b','b'], ['','',''])
    if doOkvis:
        plotterVel.addDataToSubplotMultiple(td_okvis, 'vel', [1,2,3], ['g','g','g'], ['','',''])
   
if plotAtt: # Attitude plotting
    if doRovio:
        plotterAtt.addDataToSubplotMultiple(td_rovio, 'att', [1,2,3,4], ['r','r','r','r'], ['','','',''])
    plotterAtt.addDataToSubplotMultiple(td_vicon, 'att', [1,2,3,4], ['b','b','b','b'], ['','','',''])
    if doOkvis:
        plotterAtt.addDataToSubplotMultiple(td_okvis, 'att', [1,2,3,4], ['g','g','g','g'], ['','','',''])
   
if plotRor: # Rotational rate plotting
    if doRovio:
        plotterRor.addDataToSubplotMultiple(td_rovio, 'ror', [1,2,3], ['r','r','r'], ['','',''])
    plotterRor.addDataToSubplotMultiple(td_vicon, 'ror', [1,2,3], ['b','b','b'], ['','',''])
    if doOkvis:
        plotterRor.addDataToSubplotMultiple(td_okvis, 'ror', [1,2,3], ['g','g','g'], ['','',''])
  
if plotYpr: # Yaw-pitch-roll plotting
    if doRovio:
        td_rovio.quaternionToYpr('att','ypr')
        td_rovio.quaternionToYprCov('att','attCov','yprCov')
        td_rovio.computeSigmaBounds('ypr','yprCov','yprSp','yprSm',3)
        plotterYpr.addDataToSubplotMultiple(td_rovio, 'ypr', [1,2,3], ['r','r','r'], ['','',''])
        plotterYpr.addDataToSubplotMultiple(td_rovio, 'yprSm', [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterYpr.addDataToSubplotMultiple(td_rovio, 'yprSp', [1,2,3], ['r--','r--','r--'], ['','',''])
    td_vicon.quaternionToYpr('att','ypr')
    plotterYpr.addDataToSubplotMultiple(td_vicon, 'ypr', [1,2,3], ['b','b','b'], ['','',''])
    if doOkvis:
        td_okvis.quaternionToYpr('att','ypr')
        plotterYpr.addDataToSubplotMultiple(td_okvis, 'ypr', [1,2,3], ['g','g','g'], ['','',''])

if plotExt: # Extrinsics Plotting
    if doRovio:
        td_rovio.computeSigmaBounds('extPos','extPosCov','extPosSp','extPosSm',3)
        plotterExt.addDataToSubplotMultiple(td_rovio, 'extPosSm', [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterExt.addDataToSubplotMultiple(td_rovio, 'extPosSp', [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterExt.addDataToSubplotMultiple(td_rovio, 'extPos', [1,2,3], ['r','r','r'], ['','',''])
  
if plotFea: # Plotting features height, TODO: zoom in & plot vs traveled distance
    if doRovio:
        plotFeaTimeEnd = 3.0
        startTime = 25.0
        startAverage = 1.0
        frequency = 20.0
        figure()
        x = [[]]
        y = [[]]
        cov = [[]]
        feaIdxID = td_rovio.getColIDs('feaIdx')
        feaPosID = td_rovio.getColIDs('feaPos')
        feaCovID = td_rovio.getColIDs('feaCov')
        for j in np.arange(nFeatures):
            newFea = td_rovio.col('pos') + Quaternion.q_rotate(Quaternion.q_inverse(td_rovio.col('att')),td_rovio.col(feaPosID[j]))
            td_rovio.applyRotationToCov(feaCovID[j], 'att', True)
            for i in np.arange(0,3):
                td_rovio.setCol(newFea[:,i],feaPosID[j][i])
              
            lastStart = 0.0
            lastID = -1
            startID = 0
            for i in np.arange(td_rovio.length()):
                if(td_rovio.d[i,td_rovio.timeID] > startTime):
                    if(td_rovio.d[i,feaIdxID[j]] < 0.0 or td_rovio.d[i,feaIdxID[j]] != lastID):
                        if len(x[-1]) > 0:
                            x.append([])
                            y.append([])
                            cov.append([])
                    if(td_rovio.d[i,feaIdxID[j]] >= 0.0):
                        if len(x[-1]) == 0:
                            lastStart = td_rovio.d[i,td_rovio.timeID]
                            lastID = td_rovio.d[i,feaIdxID[j]]
                            startID = i
                        x[-1].append(td_rovio.d[i,td_rovio.timeID]-lastStart)
                        y[-1].append(td_rovio.d[i,feaPosID[j][2]])
                        cov[-1].append(sqrt(td_rovio.d[i,feaCovID[j][8]]))
          
        axis = subplot(2,1,1)
        for i in np.arange(len(x)):
            plot(x[i],y[i],'#999999')
          
        average = 0
        averageCount = 0
        for i in np.arange(len(y)):
            for j in np.arange(int(startAverage*frequency),len(y[i])):
                average += y[i][j]
                averageCount += 1
        average = average/averageCount
          
        means = []
        meanCovs = []
        stds = []
        for j in np.arange(int(plotFeaTimeEnd*frequency)):
            values = []
            covValues = []
            for i in np.arange(len(y)):
                if(len(y[i]) > j):
                    values.append(y[i][j]-average)
                    covValues.append(cov[i][j])
            means.append(mean(values))
            meanCovs.append(mean(covValues))
            stds.append(std(values))
        plot(np.arange(plotFeaTimeEnd*frequency)/frequency,np.ones(plotFeaTimeEnd*frequency)*average,'b', lw=3, label='Estimated groundtruth')
        plot(np.arange(plotFeaTimeEnd*frequency)/frequency,average+np.array(means),'r', lw=3, label='Average')
        plot(np.arange(plotFeaTimeEnd*frequency)/frequency,average+np.array(means)+np.array(stds),'r--', lw=3, label='Measured standard deviation')
        plot(np.arange(plotFeaTimeEnd*frequency)/frequency,average+np.array(means)-np.array(stds),'r--', lw=3)
        plot(np.arange(plotFeaTimeEnd*frequency)/frequency,average+np.array(means)+np.array(meanCovs),'g--', lw=3, label='Estimated standard deviation')
        plot(np.arange(plotFeaTimeEnd*frequency)/frequency,average+np.array(means)-np.array(meanCovs),'g--', lw=3)
        plt.axis([0, plotFeaTimeEnd, -4.0, -1.0])
        plt.legend(loc=4)
        axis.set_ylabel('Estimated landmark height')
          
        axis = subplot(2,1,2)
        line_th = plot([0, plotFeaTimeEnd], [sqrt(6.64), sqrt(6.64)],'g--', lw=3, label='1% threshold')
        for i in np.arange(len(x)):
            plot(x[i],np.abs(np.array(y[i])-average)/np.array(cov[i]),'#999999')
        plt.axis([0, plotFeaTimeEnd, 0, 5])
        plt.legend()
        plt.suptitle('Convergence of Landmark Distance')
        axis.set_ylabel('Normalized error')
        axis.set_xlabel('Time [s]')
          
if len(plotLeutiDistances) > 0: # Leute Score evaluation
    spacings = plotLeutiDistances
    if leutiSpacing > 0:
        spacings = np.ones(len(plotLeutiDistances))*leutiSpacing
    if doRovio:
        leutiOutputRovio = td_rovio.computeLeutiScore('pos', 'att', 'vel', td_vicon, 'pos', 'att', plotLeutiDistances, spacings, 0.0)
        figure()
        ax = axes()
        p = np.arange(len(plotLeutiDistances))
        boxplot(leutiOutputRovio, positions = p, widths = 0.8)
        ax.set_xticklabels(plotLeutiDistances)
        ax.set_xlabel('Travelled Distance [m]')
        ax.set_ylabel('Position Error [m]')
        med = np.zeros(len(leutiOutputRovio))
        for i in range(len(leutiOutputRovio)):
            med[i] = median(leutiOutputRovio[i])
        s = np.array(plotLeutiDistances)**0.5
        a = s.dot(med)/s.dot(s)
        fit = a*s
        plot(p,fit)
        title('Error Plot for Position, a = ' + str(a))
    if doOkvis:
        leutiOutputOkvis = td_okvis.computeLeutiScore('pos', 'att', 'vel', td_vicon, 'pos', 'att', plotLeutiDistances, spacings, 0.0)
        figure()
        ax = axes()
        boxplot(leutiOutputOkvis, positions = np.arange(len(plotLeutiDistances)), widths = 0.8)
        ax.set_xticklabels(plotLeutiDistances)
        ax.set_xlabel('Travelled Distance [m]')
        ax.set_ylabel('Position Error [m]')
        med = np.zeros(len(leutiOutputOkvis))
        for i in range(len(leutiOutputOkvis)):
            med[i] = median(leutiOutputOkvis[i])
        s = np.array(plotLeutiDistances)**0.5
        a = s.dot(med)/s.dot(s)
        fit = a*s
        plot(p,fit)
        title('Error Plot for Position, a = ' + str(a))
         
# plt.ion()            
# plt.show(block=False)
# if True:
#     bag = rb.Bag(rovioOutputBag)
#     l1 = 0
#     l2 = 3
#     nLevels = l2-l1+1
#     figureIds = []
#     errors = []
#     gradientNorm = []
#     for l in np.arange(nLevels):
#         figureIds.append(figure().number)
#         errors.append([])
#         gradientNorm.append([])
#     patchSize = 8
#     patch = np.zeros([nLevels,patchSize,patchSize,3])
#     error = np.zeros([nLevels,patchSize,patchSize,3])
#     for top, msg, t in bag.read_messages(topics=['/rovio/patch']):
#         idField, = [x for x in msg.fields if x.name == 'id']
#         patchField, = [x for x in msg.fields if x.name == 'patch']
#         dxField, = [x for x in msg.fields if x.name == 'dx']
#         dyField, = [x for x in msg.fields if x.name == 'dy']
#         errorField, = [x for x in msg.fields if x.name == 'error']
#         step = msg.point_step
#         for i in np.arange(1):
#             idValue, = struct.unpack('i', msg.data[i*step+idField.offset:i*step+idField.offset+4])
#             if(idValue >= 0):
#                 for l in np.arange(nLevels):
#                     for y in np.arange(8):
#                         for x in np.arange(8):
#                             patchValue, = struct.unpack('f', msg.data[i*step+patchField.offset+(patchSize*patchSize*l+patchSize*y+x)*4:i*step+patchField.offset+(patchSize*patchSize*l+patchSize*y+x+1)*4])
#                             dxValue, = struct.unpack('f', msg.data[i*step+dxField.offset+(patchSize*patchSize*l+patchSize*y+x)*4:i*step+dxField.offset+(patchSize*patchSize*l+patchSize*y+x+1)*4])
#                             dyValue, = struct.unpack('f', msg.data[i*step+dyField.offset+(patchSize*patchSize*l+patchSize*y+x)*4:i*step+dyField.offset+(patchSize*patchSize*l+patchSize*y+x+1)*4])
#                             patchError, = struct.unpack('f', msg.data[i*step+errorField.offset+(patchSize*patchSize*l+patchSize*y+x)*4:i*step+errorField.offset+(patchSize*patchSize*l+patchSize*y+x+1)*4])
#                             patch[l,x,y,:] = np.ones(3,np.float32)*patchValue/255.0
#                             error[l,x,y,:] = np.ones(3,np.float32)*fabs(patchError)*10/255.0
#                             errors[l].append(patchError)
#                             gradientNorm[l].append(sqrt(dxValue*dxValue+dyValue*dyValue))
#     #                 figure(figureIds[l])
#     #                 plt.clf()
#     #                 imgplot = plt.imshow(error[l,:,:,:], interpolation="nearest")
#     #                 plt.draw()
#     fig = figure()
#     nBinsE = 100
#     nBinsG = 5
#     H, xedges, yedges = np.histogram2d(errors[1],gradientNorm[1],[nBinsE,nBinsG])
#     X, Y = np.meshgrid(0.5*(yedges[0:nBinsG]+yedges[1:nBinsG+1]),0.5*(xedges[0:nBinsE]+xedges[1:nBinsE+1]))
#     for i in range(nBinsG):
#         H[:,i] = H[:,i]/np.sum(H[:,i])*nBinsE/(xedges[-1]-xedges[0])
#     ax = fig.gca(projection='3d')
#     ax.plot_surface(X, Y, H, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False)
#      
#     fig = figure()
#     plot(errors[1],gradientNorm[1],'+')
#      
#     fig = figure()
#     ax = fig.add_subplot(111, projection='3d')
#     xedges_red = 0.5*(xedges[0:nBinsE]+xedges[1:nBinsE+1])
#     for i in range(nBinsG):
#         c1=(1.0*i/(nBinsG-1), 0.7, 1.0-1.0*i/(nBinsG-1), 1.0)
#         c2=(0.5, 0.5, 0.5, 1.0)
#         ind, = np.nonzero(np.logical_and(np.array(gradientNorm[1]) >= yedges[i],np.array(gradientNorm[1]) < yedges[i+1]))
#         sigma = std(np.take(errors[1],ind))
#         print(sigma)
#         gaussian_fit = 1/(sigma * np.sqrt(2 * np.pi)) * np.exp( - (xedges_red)**2 / (2 * sigma**2) )
#         ax.bar(xedges_red, H[:,i], width=1, zs=0.5*(yedges[i]+yedges[i+1]), zdir='y', color=c1, edgecolor=c1)
#         ax.bar(xedges_red, np.maximum(gaussian_fit-H[:,i],np.zeros(nBinsE)), width=1, zs=0.5*(yedges[i]+yedges[i+1]), zdir='y', color=c2, edgecolor=c2, bottom=H[:,i])

rovio_output = open(rovioOutputBag[:-4] + '.pkl', 'wb')
pickle.dump(td_rovio, rovio_output)

raw_input("Press Enter to continue...")
