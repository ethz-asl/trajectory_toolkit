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

# File names and settings
ID = 0

if ID == 0: # MH_01_easy with OKVIS
    rovioOutputBag = '/home/michael/datasets/euroc/MH_01_easy/rovio/2015-12-18-08-51-27.bag'
    rovioOutputTopic = '/rovio/odometry'
    okvisOutputFile = '/home/michael/datasets/euroc/MH_01_easy/okvis/2015-12-22-15-49-07.bag'
    okvisOutputTopic = '/okvis/okvis_node/okvis_transform'
    viconGroundtruthFile = '/home/michael/datasets/euroc/MH_01_easy/data.csv'
    viconGroundtruthTopic = ''
    startcut = 40
    endcut = 10
    MrMV = np.array([0.0, 0.0, 0.0])
    qVM = np.array([1.0, 0, 0, 0])
    bodyTransformForBetterPlotRangePos = np.zeros(3)
    bodyTransformForBetterPlotRangeAtt = Quaternion.q_mult(np.array([(1-0.6*0.6)**(1./2),0,0.6,0]),np.array([0.0,1.0,0.0,0.0]))
    bodyAlignViconToRovio = False
    bodyAlignOkvisToVicon = False
    doRovio = True
    doOkvis = True
    plotRon = False
    plotAtt = False
    plotPos = True
    plotVel = True
    plotRor = True
    plotYpr = True
    plotFea = False
    plotLeutiDistances = np.arange(1,21)

if ID == 1: # MH_05_difficult
    rovioOutputBag = '/home/michael/datasets/euroc/MH_05_difficult/rovio/2015-12-22-08-53-51.bag'
    rovioOutputTopic = '/rovio/odometry'
    okvisOutputFile = ''
    okvisOutputTopic = ''
    viconGroundtruthFile = '/home/michael/datasets/euroc/MH_05_difficult/data.csv'
    viconGroundtruthTopic = ''
    startcut = 20
    endcut = 10
    MrMV = np.array([0.0, 0.0, 0.0])
    qVM = np.array([1.0, 0, 0, 0])
    bodyTransformForBetterPlotRangePos = np.zeros(3)
    bodyTransformForBetterPlotRangeAtt = Quaternion.q_mult(np.array([(1-0.6*0.6)**(1./2),0,0.6,0]),np.array([0.0,1.0,0.0,0.0]))
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
    plotLeutiDistances = np.arange(1,21)

if ID == 2: # V1_03_difficult with OKVIS
    rovioOutputBag = '/home/michael/datasets/euroc/V1_03_difficult/rovio/2015-12-21-17-21-32.bag'
    rovioOutputTopic = '/rovio/odometry'
    okvisOutputFile = '/home/michael/datasets/euroc/V1_03_difficult/okvis/2015-12-22-16-11-13.bag'
    okvisOutputTopic = '/okvis/okvis_node/okvis_transform'
    viconGroundtruthFile = '/home/michael/datasets/euroc/V1_03_difficult/data.csv'
    viconGroundtruthTopic = ''
    startcut = 0
    endcut = 0
    MrMV = np.array([0.0, 0.0, 0.0])
    qVM = np.array([1.0, 0, 0, 0])
    bodyTransformForBetterPlotRangePos = np.zeros(3)
    bodyTransformForBetterPlotRangeAtt = Quaternion.q_mult(np.array([(1-0.6*0.6)**(1./2),0,0.6,0]),np.array([0.0,1.0,0.0,0.0]))
    MtMV = -3.80322e-02
    bodyAlignViconToRovio = False
    bodyAlignOkvisToVicon = False
    doRovio = True
    doOkvis = True
    plotRon = False
    plotAtt = False
    plotPos = True
    plotVel = True
    plotRor = True
    plotYpr = True
    plotFea = False
    plotLeutiDistances = np.arange(1,21)

if ID == 3: # FlyingWithBurri/rounds_6
    rovioOutputBag = '/home/michael/datasets/FlyingWithBurri/rounds_6/2015-11-17-14-56-45.bag'
    rovioOutputTopic = '/rovio/odometry'
    okvisOutputFile = ''
    okvisOutputTopic = ''
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
    plotRon = False
    plotAtt = False
    plotPos = True
    plotVel = True
    plotRor = True
    plotYpr = True
    plotFea = False
    plotLeutiDistances = np.arange(1,21)

if ID == 4: # bloesch_leo/planar_1
    rovioOutputBag = '/home/michael/datasets/bloesch_leo/planar_1/2015-12-23-17-24-06.bag'
    rovioOutputTopic = '/rovio/odometry'
    okvisOutputFile = ''
    okvisOutputTopic = ''
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
    plotRon = False
    plotAtt = False
    plotPos = True
    plotVel = False
    plotRor = False
    plotYpr = True
    plotFea = True
    plotLeutiDistances = []

if ID == 5: # Long Dataset
    rovioOutputBag = '/home/michael/datasets/long_trajectory_leutiLynen/rovio/2015-12-23-09-42-47.bag'
    rovioOutputTopic = '/rovio/odometry'
    okvisOutputFile = '/home/michael/datasets/long_trajectory_leutiLynen/aslam/aslam_estimator_output_data.csv'
    okvisOutputTopic = ''
    viconGroundtruthFile = '/home/michael/datasets/long_trajectory_leutiLynen/long_trajectory.bag'
    viconGroundtruthTopic = '/vicon/sensor_cross/sensor_cross'
    startcut = 0
    endcut = 0
    MrMV = np.array([0.0, 0.0, 0.0])
    qVM = np.array([1.0, 0, 0, 0])
    bodyTransformForBetterPlotRangePos = np.zeros(3)
    bodyTransformForBetterPlotRangeAtt = np.array([1.0,0.0,0.0,0.0])
    bodyAlignViconToRovio = True
    bodyAlignOkvisToVicon = True
    doRovio = True
    doOkvis = True
    plotRon = False
    plotAtt = False
    plotPos = True
    plotVel = True
    plotRor = True
    plotYpr = True
    plotFea = False
    plotLeutiDistances = [10,40,90,160,250,360]

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
    if plotFea:
        plotterFea = Plotter(-1, [3,1],'Feature Tracks',['','','time[s]'],['x[m]','y[m]','z[m]'],10000)

if doRovio: # Rovio Timed Data
    nFeatures = 25
    td_rovio = TimedData(58+nFeatures*4)
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
    rovio_fea_posID = []
    for i in np.arange(nFeatures):
        rovio_fea_posID.append([58+3*i,59+3*i,60+3*i])
    rovio_fea_idxID = np.arange(nFeatures)+58+nFeatures*3

if True: # Vicon Timed Data
    td_vicon = TimedData(18)
    vicon_posID = [1,2,3]
    vicon_attID = [4,5,6,7]
    vicon_velID = [8,9,10]
    vicon_rorID = [11,12,13]
    vicon_ronID = 14
    vicon_yprID = [15,16,17]

if doOkvis: # Okvis Timed Data
    td_okvis = TimedData(18)
    okvis_posID = [1,2,3]
    okvis_attID = [4,5,6,7]
    okvis_velID = [8,9,10]
    okvis_rorID = [11,12,13]
    okvis_ronID = 14
    okvis_yprID = [15,16,17]

if True: # Vicon data acquisition and pre-processing
    if(viconGroundtruthFile.endswith('.csv')):
        CsvDataAcquisition.csvLoadTransform(viconGroundtruthFile, 0, 1, 4, td_vicon, vicon_posID, vicon_attID)
    elif(viconGroundtruthFile.endswith('.bag')):
        RosDataAcquisition.rosBagLoadTransformStamped(viconGroundtruthFile,viconGroundtruthTopic,td_vicon,vicon_posID,vicon_attID)
    if(viconGroundtruthFile.endswith('data.csv')):
        td_vicon.d[:,0] = td_vicon.d[:,0]*1e-9
    td_vicon.cropTimes(td_vicon.getFirstTime()+startcut,td_vicon.getLastTime()-endcut)
    td_vicon.applyTimeOffset(-td_vicon.getFirstTime())
    td_vicon.computeRotationalRateFromAttitude(vicon_attID,vicon_rorID)
    td_vicon.computeNormOfColumns(vicon_rorID,vicon_ronID)
    td_vicon.computeVelocitiesInBodyFrameFromPostionInWorldFrame(vicon_posID, vicon_velID, vicon_attID)

if doRovio: # Rovio data acquisition and pre-processing
    RosDataAcquisition.rosBagLoadOdometry(rovioOutputBag, rovioOutputTopic ,td_rovio,rovio_posID,rovio_attID,rovio_velID,rovio_rorID,rovio_posCovID,rovio_attCovID)
    RosDataAcquisition.rosBagLoadRobocentricPointCloud(rovioOutputBag,'/rovio/pcl',td_rovio,rovio_fea_idxID,rovio_fea_posID)
    td_rovio.computeNormOfColumns(rovio_rorID,rovio_ronID)
    td_rovio.applyTimeOffset(td_vicon.getFirstTime()-td_rovio.getFirstTime())
    to = td_rovio.getTimeOffset(rovio_ronID,td_vicon,vicon_ronID)
    td_rovio.applyTimeOffset(-to)
    td_rovio.cropTimes(td_vicon.getFirstTime(),td_vicon.getLastTime())

if doOkvis: # Okvis data acquisition and pre-processing
    if(okvisOutputFile.endswith('.csv')):
        CsvDataAcquisition.csvLoadTransform(okvisOutputFile, 0, 1, 4, td_okvis, okvis_posID, okvis_attID)
    elif(okvisOutputFile.endswith('.bag')):
        RosDataAcquisition.rosBagLoadTransformStamped(okvisOutputFile,okvisOutputTopic,td_okvis,okvis_posID,okvis_attID)
    if(okvisOutputFile.endswith('data.csv')):
        td_okvis.d[:,0] = td_okvis.d[:,0]*1e-9
    if(okvisOutputFile.endswith('0.5speed.bag')):
        td_okvis.d[:,0] = 0.5*td_okvis.d[:,0]
    if(okvisOutputFile == '/home/michael/datasets/long_trajectory_leutiLynen/aslam/aslam_estimator_output_data.csv'):
        q_real = td_okvis.col(okvis_attID[3])
        q_im = -td_okvis.cols(okvis_attID[0:3])
        td_okvis.setCols(q_real,okvis_attID[0])
        td_okvis.setCols(q_im,okvis_attID[1:4])
    td_okvis.computeRotationalRateFromAttitude(okvis_attID,okvis_rorID,2,2)
    td_okvis.computeNormOfColumns(okvis_rorID,okvis_ronID)
    td_okvis.applyTimeOffset(td_vicon.getFirstTime()-td_okvis.getFirstTime())
    to = td_okvis.getTimeOffset(okvis_ronID,td_vicon,vicon_ronID)
    td_okvis.applyTimeOffset(-to)
    td_okvis.cropTimes(td_vicon.getFirstTime(),td_vicon.getLastTime())
    td_okvis.computeVelocitiesInBodyFrameFromPostionInWorldFrame(okvis_posID, okvis_velID, okvis_attID)
        
if plotRon: # Plotting rotational rate norm
    if doRovio:
        plotterRon.addDataToSubplot(td_rovio, rovio_ronID, 1, 'r', 'rovio rotational rate norm')
    plotterRon.addDataToSubplot(td_vicon, vicon_ronID, 1, 'b', 'vicon rotational rate norm')
    if doOkvis:
        plotterRon.addDataToSubplot(td_okvis, okvis_ronID, 1, 'g', 'okvis rotational rate norm')

if True: # Transform body coordinate frame for better vizualization, TODO: add to config
    if doRovio:
        td_rovio.applyBodyTransform(rovio_posID, rovio_attID, bodyTransformForBetterPlotRangePos, bodyTransformForBetterPlotRangeAtt)
        td_rovio.applyBodyTransformToAttCov(rovio_attCovID, bodyTransformForBetterPlotRangeAtt)
        td_rovio.applyBodyTransformToTwist(rovio_velID, rovio_rorID, bodyTransformForBetterPlotRangePos, bodyTransformForBetterPlotRangeAtt)
        td_rovio.applyInertialTransform(rovio_posID, rovio_attID, np.zeros(3), np.array([0,0,0,1]))
    if doOkvis:
        td_okvis.applyBodyTransform(okvis_posID, okvis_attID, bodyTransformForBetterPlotRangePos, bodyTransformForBetterPlotRangeAtt)
        td_okvis.applyBodyTransformToTwist(okvis_velID, okvis_rorID, bodyTransformForBetterPlotRangePos, bodyTransformForBetterPlotRangeAtt)
        td_okvis.applyInertialTransform(okvis_posID, okvis_attID, np.zeros(3), np.array([0,0,0,1]))
        
if True: # Align Vicon to Rovio using calibration (body frame)
    B_r_BC_est = -Quaternion.q_rotate(qVM, MrMV) + Quaternion.q_rotate(Quaternion.q_inverse(qVM),bodyTransformForBetterPlotRangePos)
    qCB_est = Quaternion.q_mult(bodyTransformForBetterPlotRangeAtt,Quaternion.q_inverse(qVM))
    td_vicon.applyBodyTransform(vicon_posID, vicon_attID, B_r_BC_est, qCB_est)
    td_vicon.applyBodyTransformToTwist(vicon_velID, vicon_rorID, B_r_BC_est, qCB_est)
 
if doRovio and bodyAlignViconToRovio: # Align Vicon to Rovio (body frame)
    B_r_BC_est, qCB_est = td_vicon.calibrateBodyTransform(vicon_velID, vicon_rorID, td_rovio, rovio_velID,rovio_rorID)
    print('Calibrate Body Transform for Rovio:')
    print('Quaternion Rotation qCB_est:\tw:' + str(qCB_est[0]) + '\tx:' + str(qCB_est[1]) + '\ty:' + str(qCB_est[2]) + '\tz:' + str(qCB_est[3]))
    print('Translation Vector B_r_BC_est:\tx:' + str(B_r_BC_est[0]) + '\ty:' + str(B_r_BC_est[1]) + '\tz:' + str(B_r_BC_est[2]))
    td_vicon.applyBodyTransform(vicon_posID, vicon_attID, B_r_BC_est, qCB_est)
    td_vicon.applyBodyTransformToTwist(vicon_velID, vicon_rorID, B_r_BC_est, qCB_est)

if doOkvis and bodyAlignOkvisToVicon: # Align Okvis to Vicon (body frame)
    B_r_BC_est, qCB_est = td_okvis.calibrateBodyTransform(okvis_velID, okvis_rorID, td_vicon, vicon_velID,vicon_rorID)
    print('Calibrate Body Transform for Okvis:')
    print('Quaternion Rotation qCB_est:\tw:' + str(qCB_est[0]) + '\tx:' + str(qCB_est[1]) + '\ty:' + str(qCB_est[2]) + '\tz:' + str(qCB_est[3]))
    print('Translation Vector B_r_BC_est:\tx:' + str(B_r_BC_est[0]) + '\ty:' + str(B_r_BC_est[1]) + '\tz:' + str(B_r_BC_est[2]))
    td_okvis.applyBodyTransform(okvis_posID, okvis_attID, B_r_BC_est, qCB_est)
    td_okvis.applyBodyTransformToTwist(okvis_velID, okvis_rorID, B_r_BC_est, qCB_est)
    
if doRovio: # Align Vicon to Rovio (Inertial frames)
    J_r_JI_est, qIJ_est = td_vicon.calibrateInertialTransform(vicon_posID, vicon_attID, td_rovio, rovio_posID,rovio_attID, np.array([0.0,0.0,0.0]), np.array([1.0,0.0,0.0,0.0]), [0,1,2,3,4,5])
    print('Calibrate Inertial Transform for Rovio:')
    print('uaternion Rotation qIJ_est:\tw:' + str(qIJ_est[0]) + '\tx:' + str(qIJ_est[1]) + '\ty:' + str(qIJ_est[2]) + '\tz:' + str(qIJ_est[3]))
    print('Translation Vector J_r_JI_est:\tx:' + str(J_r_JI_est[0]) + '\ty:' + str(J_r_JI_est[1]) + '\tz:' + str(J_r_JI_est[2]))
    td_vicon.applyInertialTransform(vicon_posID, vicon_attID,J_r_JI_est,qIJ_est)

if doOkvis: # Align Okvis to Vicon (Inertial frames)
    J_r_JI_est, qIJ_est = td_okvis.calibrateInertialTransform(okvis_posID, okvis_attID, td_vicon, vicon_posID,vicon_attID, np.array([0.0,0.0,0.0]), np.array([1.0,0.0,0.0,0.0]), [0,1,2,3,4,5])
    print('Calibrate Inertial Transform for Okvis:')
    print('uaternion Rotation qIJ_est:\tw:' + str(qIJ_est[0]) + '\tx:' + str(qIJ_est[1]) + '\ty:' + str(qIJ_est[2]) + '\tz:' + str(qIJ_est[3]))
    print('Translation Vector J_r_JI_est:\tx:' + str(J_r_JI_est[0]) + '\ty:' + str(J_r_JI_est[1]) + '\tz:' + str(J_r_JI_est[2]))
    td_okvis.applyInertialTransform(okvis_posID, okvis_attID,J_r_JI_est,qIJ_est)

if plotPos: # Position plotting
    if doRovio:
        td_rovio.computeSigmaBounds(rovio_posID,[rovio_posCovID[0],rovio_posCovID[4],rovio_posCovID[8]],rovio_posSpID,rovio_posSmID,3)
        plotterPos.addDataToSubplotMultiple(td_rovio, rovio_posID, [1,2,3], ['r','r','r'], ['','',''])
        plotterPos.addDataToSubplotMultiple(td_rovio, rovio_posSmID, [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterPos.addDataToSubplotMultiple(td_rovio, rovio_posSpID, [1,2,3], ['r--','r--','r--'], ['','',''])
    plotterPos.addDataToSubplotMultiple(td_vicon, vicon_posID, [1,2,3], ['b','b','b'], ['','',''])
    if doOkvis:
        plotterPos.addDataToSubplotMultiple(td_okvis, okvis_posID, [1,2,3], ['g','g','g'], ['','',''])
 
if plotVel: # Velocity plotting
    if doRovio:
        plotterVel.addDataToSubplotMultiple(td_rovio, rovio_velID, [1,2,3], ['r','r','r'], ['','',''])
    plotterVel.addDataToSubplotMultiple(td_vicon, vicon_velID, [1,2,3], ['b','b','b'], ['','',''])
    if doOkvis:
        plotterVel.addDataToSubplotMultiple(td_okvis, okvis_velID, [1,2,3], ['g','g','g'], ['','',''])
 
if plotAtt: # Attitude plotting
    if doRovio:
        plotterAtt.addDataToSubplotMultiple(td_rovio, rovio_attID, [1,2,3,4], ['r','r','r','r'], ['','','',''])
    plotterAtt.addDataToSubplotMultiple(td_vicon, vicon_attID, [1,2,3,4], ['b','b','b','b'], ['','','',''])
    if doOkvis:
        plotterAtt.addDataToSubplotMultiple(td_okvis, okvis_attID, [1,2,3,4], ['g','g','g','g'], ['','','',''])
 
if plotRor: # Rotational rate plotting
    if doRovio:
        plotterRor.addDataToSubplotMultiple(td_rovio, rovio_rorID, [1,2,3], ['r','r','r'], ['','',''])
    plotterRor.addDataToSubplotMultiple(td_vicon, vicon_rorID, [1,2,3], ['b','b','b'], ['','',''])
    if doOkvis:
        plotterRor.addDataToSubplotMultiple(td_okvis, okvis_rorID, [1,2,3], ['g','g','g'], ['','',''])

if plotYpr: # Yaw-pitch-roll plotting
    if doRovio:
        td_rovio.quaternionToYpr(rovio_attID,rovio_yprID)
        td_rovio.quaternionToYprCov(rovio_attID,rovio_attCovID,rovio_yprCovID)
        td_rovio.computeSigmaBounds(rovio_yprID,[rovio_yprCovID[0],rovio_yprCovID[4],rovio_yprCovID[8]],rovio_yprSpID,rovio_yprSmID,3)
        plotterYpr.addDataToSubplotMultiple(td_rovio, rovio_yprID, [1,2,3], ['r','r','r'], ['','',''])
        plotterYpr.addDataToSubplotMultiple(td_rovio, rovio_yprSmID, [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterYpr.addDataToSubplotMultiple(td_rovio, rovio_yprSpID, [1,2,3], ['r--','r--','r--'], ['','',''])
    td_vicon.quaternionToYpr(vicon_attID,vicon_yprID)
    plotterYpr.addDataToSubplotMultiple(td_vicon, vicon_yprID, [1,2,3], ['b','b','b'], ['','',''])
    if doOkvis:
        td_okvis.quaternionToYpr(okvis_attID,okvis_yprID)
        plotterYpr.addDataToSubplotMultiple(td_okvis, okvis_yprID, [1,2,3], ['g','g','g'], ['','',''])

if plotFea: # Plotting rotational rate norm
    if doRovio:
        figure()
        x = [[]]
        y = [[]]
        for j in np.arange(nFeatures):
            newFea = td_rovio.cols(rovio_posID) + Quaternion.q_rotate(Quaternion.q_inverse(td_rovio.cols(rovio_attID)),td_rovio.cols(rovio_fea_posID[j]))
            for i in np.arange(0,3):
                td_rovio.setCol(newFea[:,i],rovio_fea_posID[j][i])
            
            lastStart = 0.0
            lastID = -1
            startID = 0
            for i in np.arange(td_rovio.length()):
                if(td_rovio.d[i,td_rovio.timeID] > 25.0):
                    if(td_rovio.d[i,rovio_fea_idxID[j]] < 0.0 or td_rovio.d[i,rovio_fea_idxID[j]] != lastID):
                        if len(x[-1]) > 0:
                            x.append([])
                            y.append([])
                    if(td_rovio.d[i,rovio_fea_idxID[j]] >= 0.0):
                        if len(x[-1]) == 0:
                            lastStart = td_rovio.d[i,td_rovio.timeID]
                            lastID = td_rovio.d[i,rovio_fea_idxID[j]]
                            startID = i
                        x[-1].append(td_rovio.d[i,td_rovio.timeID]-lastStart)
                        y[-1].append(td_rovio.d[i,rovio_fea_posID[j][2]])
            
        for i in np.arange(len(x)):
            plot(x[i],y[i],'#999999')
        
        average = 0
        averageCount = 0
        for i in np.arange(len(y)):
            for j in np.arange(20,len(y[i])):
                average += y[i][j]
                averageCount += 1
        average = average/averageCount
        
        means = []
        stds = []
        for j in np.arange(200):
            values = []
            for i in np.arange(len(y)):
                if(len(y[i]) > j):
                    values.append(y[i][j]-average)
            means.append(mean(values))
#             stds.append((mean(np.array(values)**(2.0)))**0.5) # TODO
            stds.append(std(values))
        plot(np.arange(200)*0.05,np.ones(200)*average,'b', lw=3)
        plot(np.arange(200)*0.05,average+np.array(means),'r', lw=3)
        plot(np.arange(200)*0.05,average+np.array(means)+np.array(stds),'r--', lw=3)
        plot(np.arange(200)*0.05,average+np.array(means)-np.array(stds),'r--', lw=3)
        
if len(plotLeutiDistances) > 0: # Leute Score evaluation
    spacings = plotLeutiDistances
    if doRovio:
        leutiOutputRovio = td_rovio.computeLeutiScore(rovio_posID, rovio_attID, rovio_velID, td_vicon, vicon_posID, vicon_attID, plotLeutiDistances, spacings, 0.0)
        figure()
        ax = axes()
        boxplot(leutiOutputRovio, positions = np.arange(len(plotLeutiDistances)), widths = 0.8)
        ax.set_xticklabels(plotLeutiDistances)
        ax.set_xlabel('Travelled Distance [m]')
        ax.set_ylabel('Position Error [m]')
    if doOkvis:
        leutiOutputOkvis = td_okvis.computeLeutiScore(okvis_posID, okvis_attID, okvis_velID, td_vicon, vicon_posID, vicon_attID, plotLeutiDistances, spacings, 0.0)
        figure()
        ax = axes()
        boxplot(leutiOutputOkvis, positions = np.arange(len(plotLeutiDistances)), widths = 0.8)
        ax.set_xticklabels(plotLeutiDistances)
        ax.set_xlabel('Travelled Distance [m]')
        ax.set_ylabel('Position Error [m]')

raw_input("Press Enter to continue...")
