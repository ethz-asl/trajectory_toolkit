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
    plotLeuti = True

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
    plotLeuti = True

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
    plotLeuti = True

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
    plotLeuti = True

if ID == 4: # bloesch_leo/planar_1
    rovioOutputBag = '/home/michael/datasets/bloesch_leo/planar_1/2015-12-22-15-13-09.bag'
    rovioOutputTopic = '/rovio/odometry'
    okvisOutputFile = ''
    okvisOutputTopic = ''
    viconGroundtruthFile = '/home/michael/datasets/bloesch_leo/planar_1/2015-12-22-14-15-42.bag'
    viconGroundtruthTopic = '/bluebird/vrpn_client/estimated_transform'
    startcut = 0
    endcut = 0
    MrMV = np.array([6.90120e-02, -2.78077e-02, -1.23948e-01]) # TODO: check
    qVM = np.array([1.42930e-03, 8.17427e-01, -1.17036e-02, 5.75911e-01])
    bodyAlignViconToRovio = False
    bodyAlignOkvisToVicon = False
    doRovio = True
    doOkvis = False
    plotRon = False
    plotAtt = False
    plotPos = False
    plotVel = False
    plotRor = False
    plotYpr = False
    plotLeuti = False

if True: # Plotter initialization
    if plotRon:
        plotterRon = Plotter(1, [1,1])
    if plotAtt:
        plotterAtt = Plotter(2, [4,1])
    if plotPos:
        plotterPos = Plotter(3, [3,1])
    if plotVel:
        plotterVel = Plotter(4, [3,1])
    if plotRor:
        plotterRor = Plotter(5, [3,1])
    if plotYpr:
        plotterYpr = Plotter(6, [3,1])

if doRovio: # Rovio Timed Data
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
        CsvDataAcquisition.csvLoadTransform(viconGroundtruthFile, 0, 1, 4, td_vicon, vicon_posID[0], vicon_attID[0])
    elif(viconGroundtruthFile.endswith('.bag')):
        RosDataAcquisition.rosBagLoadTransformStamped(viconGroundtruthFile,viconGroundtruthTopic,td_vicon,vicon_posID[0],vicon_attID[0])
    if(viconGroundtruthFile.endswith('data.csv')):
        td_vicon.d[:,0] = td_vicon.d[:,0]*1e-9
    td_vicon.cropTimes(td_vicon.getFirstTime()+startcut,td_vicon.getLastTime()-endcut)
    td_vicon.applyTimeOffset(-td_vicon.getFirstTime())
    td_vicon.computeRotationalRateFromAttitude(vicon_attID[0],vicon_rorID[0])
    td_vicon.computeNormOfColumns(vicon_rorID,vicon_ronID)
    td_vicon.computeVelocitiesInBodyFrameFromPostionInWorldFrame(vicon_posID, vicon_velID, vicon_attID)

if doRovio: # Rovio data acquisition and pre-processing
    RosDataAcquisition.rosBagLoadOdometry(rovioOutputBag, rovioOutputTopic ,td_rovio,rovio_posID[0],rovio_attID[0],rovio_velID[0],rovio_rorID[0],rovio_posCovID[0],rovio_attCovID[0])
    td_rovio.computeNormOfColumns(rovio_rorID,rovio_ronID)
    td_rovio.applyTimeOffset(td_vicon.getFirstTime()-td_rovio.getFirstTime())
    to = td_rovio.getTimeOffset(rovio_ronID,td_vicon,vicon_ronID)
    td_rovio.applyTimeOffset(-to)
    td_rovio.cropTimes(td_vicon.getFirstTime(),td_vicon.getLastTime())

if doOkvis: # Okvis data acquisition and pre-processing
    RosDataAcquisition.rosBagLoadTransformStamped(okvisOutputFile,okvisOutputTopic,td_okvis,okvis_posID[0],okvis_attID[0])
    if(okvisOutputFile.endswith('0.5speed.bag')):
        td_okvis.d[:,0] = 0.5*td_okvis.d[:,0]
    td_okvis.computeRotationalRateFromAttitude(okvis_attID[0],okvis_rorID[0],2,2)
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
    bodyTransformForBetterPlotRangePos = np.zeros(3)
    bodyTransformForBetterPlotRangeAtt = Quaternion.q_mult(np.array([(1-0.6*0.6)**(1./2),0,0.6,0]),np.array([0.0,1.0,0.0,0.0]))
    if doRovio:
        td_rovio.applyBodyTransform(rovio_posID[0], rovio_attID[0], bodyTransformForBetterPlotRangePos, bodyTransformForBetterPlotRangeAtt)
        td_rovio.applyBodyTransformToAttCov(rovio_attCovID, bodyTransformForBetterPlotRangeAtt)
        td_rovio.applyBodyTransformToTwist(rovio_velID[0], rovio_rorID[0], bodyTransformForBetterPlotRangePos, bodyTransformForBetterPlotRangeAtt)
        td_rovio.applyInertialTransform(rovio_posID[0], rovio_attID[0], np.zeros(3), np.array([0,0,0,1]))
    if doOkvis:
        td_okvis.applyBodyTransform(okvis_posID[0], okvis_attID[0], bodyTransformForBetterPlotRangePos, bodyTransformForBetterPlotRangeAtt)
        td_okvis.applyBodyTransformToTwist(okvis_velID[0], okvis_rorID[0], bodyTransformForBetterPlotRangePos, bodyTransformForBetterPlotRangeAtt)
        td_okvis.applyInertialTransform(okvis_posID[0], okvis_attID[0], np.zeros(3), np.array([0,0,0,1]))
        
if True: # Align Vicon to Rovio using calibration (body frame)
    B_r_BC_est = -Quaternion.q_rotate(qVM, MrMV) + Quaternion.q_rotate(Quaternion.q_inverse(qVM),bodyTransformForBetterPlotRangePos)
    qCB_est = Quaternion.q_mult(bodyTransformForBetterPlotRangeAtt,Quaternion.q_inverse(qVM))
    td_vicon.applyBodyTransform(vicon_posID[0], vicon_attID[0], B_r_BC_est, qCB_est)
    td_vicon.applyBodyTransformToTwist(vicon_velID[0], vicon_rorID[0], B_r_BC_est, qCB_est)
 
if doRovio and bodyAlignViconToRovio: # Align Vicon to Rovio (body frame)
    B_r_BC_est, qCB_est = td_vicon.calibrateBodyTransform(vicon_velID[0], vicon_rorID[0], td_rovio, rovio_velID[0],rovio_rorID[0])
    print('Calibrate Body Transform for Rovio:')
    print('Quaternion Rotation qCB_est:\tw:' + str(qCB_est[0]) + '\tx:' + str(qCB_est[1]) + '\ty:' + str(qCB_est[2]) + '\tz:' + str(qCB_est[3]))
    print('Translation Vector B_r_BC_est:\tx:' + str(B_r_BC_est[0]) + '\ty:' + str(B_r_BC_est[1]) + '\tz:' + str(B_r_BC_est[2]))
    td_vicon.applyBodyTransform(vicon_posID[0], vicon_attID[0], B_r_BC_est, qCB_est)
    td_vicon.applyBodyTransformToTwist(vicon_velID[0], vicon_rorID[0], B_r_BC_est, qCB_est)

if doOkvis and bodyAlignOkvisToVicon: # Align Okvis to Vicon (body frame)
    B_r_BC_est, qCB_est = td_okvis.calibrateBodyTransform(okvis_velID[0], okvis_rorID[0], td_vicon, vicon_velID[0],vicon_rorID[0])
    print('Calibrate Body Transform for Okvis:')
    print('Quaternion Rotation qCB_est:\tw:' + str(qCB_est[0]) + '\tx:' + str(qCB_est[1]) + '\ty:' + str(qCB_est[2]) + '\tz:' + str(qCB_est[3]))
    print('Translation Vector B_r_BC_est:\tx:' + str(B_r_BC_est[0]) + '\ty:' + str(B_r_BC_est[1]) + '\tz:' + str(B_r_BC_est[2]))
    td_okvis.applyBodyTransform(okvis_posID[0], okvis_attID[0], B_r_BC_est, qCB_est)
    td_okvis.applyBodyTransformToTwist(okvis_velID[0], okvis_rorID[0], B_r_BC_est, qCB_est)
    
if doRovio: # Align Vicon to Rovio (Inertial frames)
    J_r_JI_est, qIJ_est = td_vicon.calibrateInertialTransform(vicon_posID[0], vicon_attID[0], td_rovio, rovio_posID[0],rovio_attID[0], np.array([0.0,0.0,0.0]), np.array([1.0,0.0,0.0,0.0]), [0,1,2,3,4,5])
    print('Calibrate Inertial Transform for Rovio:')
    print('uaternion Rotation qIJ_est:\tw:' + str(qIJ_est[0]) + '\tx:' + str(qIJ_est[1]) + '\ty:' + str(qIJ_est[2]) + '\tz:' + str(qIJ_est[3]))
    print('Translation Vector J_r_JI_est:\tx:' + str(J_r_JI_est[0]) + '\ty:' + str(J_r_JI_est[1]) + '\tz:' + str(J_r_JI_est[2]))
    td_vicon.applyInertialTransform(vicon_posID[0], vicon_attID[0],J_r_JI_est,qIJ_est)

if doOkvis: # Align Okvis to Vicon (Inertial frames)
    J_r_JI_est, qIJ_est = td_okvis.calibrateInertialTransform(okvis_posID[0], okvis_attID[0], td_vicon, vicon_posID[0],vicon_attID[0], np.array([0.0,0.0,0.0]), np.array([1.0,0.0,0.0,0.0]), [0,1,2,3,4,5])
    print('Calibrate Inertial Transform for Okvis:')
    print('uaternion Rotation qIJ_est:\tw:' + str(qIJ_est[0]) + '\tx:' + str(qIJ_est[1]) + '\ty:' + str(qIJ_est[2]) + '\tz:' + str(qIJ_est[3]))
    print('Translation Vector J_r_JI_est:\tx:' + str(J_r_JI_est[0]) + '\ty:' + str(J_r_JI_est[1]) + '\tz:' + str(J_r_JI_est[2]))
    td_okvis.applyInertialTransform(okvis_posID[0], okvis_attID[0],J_r_JI_est,qIJ_est)

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

if plotLeuti: # Leute Score evaluation
    distances = np.arange(1,21)
    spacings = distances
    if doRovio:
        leutiOutputRovio = td_rovio.computeLeutiScore(rovio_posID, rovio_attID, rovio_velID, td_vicon, vicon_posID, vicon_attID, distances, spacings, 0.0)
        figure(7)
        ax = axes()
        boxplot(leutiOutputRovio, positions = distances, widths = 0.6)
        ax.set_xticklabels(distances)
    if doOkvis:
        leutiOutputOkvis = td_okvis.computeLeutiScore(okvis_posID, okvis_attID, okvis_velID, td_vicon, vicon_posID, vicon_attID, distances, spacings, 0.0)
        figure(8)
        ax = axes()
        boxplot(leutiOutputOkvis, positions = distances, widths = 0.6)
        ax.set_xticklabels(distances)

raw_input("Press Enter to continue...")
