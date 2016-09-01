# Imports
import os, sys, inspect
import numpy as np

from trajectory_toolkit.TimedData import TimedData
from trajectory_toolkit.Plotter import Plotter
from trajectory_toolkit import Quaternion
from trajectory_toolkit import Utils
from trajectory_toolkit import RosDataAcquisition

td = TimedData()
td_aligned = TimedData()

# Setup options
testFolder = '/home/michael/datasets/rovioEval/test'
Utils.createFolderIfMissing(testFolder)
camera = '/home/michael/datasets/long_trajectory_leutiLynen/cam0.yaml'
odometryBag = '/home/michael/datasets/euroc/V1_03_difficult/okvis/okvis_paramest_V1_03_difficult.bag'
odometryTopic = '/okvis/okvis_node/okvis_odometry'
timestampBag = '/home/michael/datasets/euroc/V1_03_difficult/okvis/okvis_paramest_V1_03_difficult.bag'
timestampTopic = '/okvis/okvis_node/okvis_odometry'
R_BS = np.array([0.0148655429818, -0.999880929698, 0.00414029679422,
     0.999557249008, 0.0149672133247, 0.025715529948,
     -0.0257744366974, 0.00375618835797, 0.999660727178])
bodyTranslation = np.array([-0.0216401454975,-0.064676986768,0.00981073058949])
bodyRotation = Quaternion.q_inverse(Quaternion.q_rotMatToQuat(R_BS))
inertialTranslation = None
inertialRotation = None
writeData = False

# Load Odometry Data
td.addLabelingIncremental('pos',3)
td.addLabelingIncremental('att',4)
td.reInit()
RosDataAcquisition.rosBagLoadOdometry(odometryBag, odometryTopic, td, 'pos','att')

# Load Timestamps into new td
td_aligned.addLabelingIncremental('pos',3)
td_aligned.addLabelingIncremental('att',4)
td_aligned.reInit()
RosDataAcquisition.rosBagLoadTimestampsOnly(timestampBag,timestampTopic,td_aligned)

# Interpolate Data
td.interpolateColumns(td_aligned, 'pos', 'pos')
td.interpolateQuaternion(td_aligned, 'att', 'att')

# Plotting
plotterPos = Plotter(-1, [3,1],'Position',['','','time[s]'],['x[m]','y[m]','z[m]'],10000)
plotterAtt = Plotter(-1, [4,1],'Attitude',['','','','time[s]'],['w','x','y','z'],10000)
plotterPos.addDataToSubplotMultiple(td, 'pos', [1,2,3], ['b','b','b'], ['','',''])
plotterAtt.addDataToSubplotMultiple(td, 'att', [1,2,3,4], ['b','b','b','b'], ['','','',''])
plotterPos.addDataToSubplotMultiple(td_aligned, 'pos', [1,2,3], ['g','g','g'], ['','',''])
plotterAtt.addDataToSubplotMultiple(td_aligned, 'att', [1,2,3,4], ['g','g','g','g'], ['','','',''])

# Apply body and inertial 
if bodyTranslation != None and bodyRotation != None:
    td_aligned.applyBodyTransform('pos', 'att', bodyTranslation, bodyRotation)
if inertialTranslation != None and inertialRotation != None:
    td_aligned.applyBodyTransform('pos', 'att', inertialTranslation, inertialRotation)
plotterPos.addDataToSubplotMultiple(td_aligned, 'pos', [1,2,3], ['r','r','r'], ['','',''])
plotterAtt.addDataToSubplotMultiple(td_aligned, 'att', [1,2,3,4], ['r','r','r','r'], ['','','',''])

if writeData:
    td_aligned.writeColsToSingleFiles(testFolder + '/poseOut', 'pose', td_aligned.getColIDs('pos') + td_aligned.getColIDs('att'), ' ')

raw_input("Press Enter to continue...")
