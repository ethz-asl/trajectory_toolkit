# Imports
import os, sys, inspect getopt
import numpy as np

from trajectory_toolkit.TimedData import TimedData
from trajectory_toolkit.Plotter import Plotter
from trajectory_toolkit import Quaternion
from trajectory_toolkit import Utils
from trajectory_toolkit import RosDataAcquisition

td = TimedData()
td_aligned = TimedData()

# Setup options
testFolder = '/home/michael/datasets/imperial/190416_Lukas'
odometryBag = '/home/michael/datasets/imperial/190416_Lukas/2016-04-19-16-31-38_okvis.bag'
odometryTopic = '/okvis_node/okvis_odometry'
timestampBag = '/home/michael/datasets/imperial/190416_Lukas/2016-04-19-16-31-38.bag'
timestampTopic = '/cam0/image_raw'
R_BC = np.array([0.99999316, 0.0036546, -0.00057556,
          -0.0036534, 0.99999119, 0.00206604,
          0.00058311, -0.00206393, 0.9999977])
bodyTranslation = np.array([0.0363702,-0.00575547,-0.0028041]) # B_r_BC
bodyRotation = Quaternion.q_inverse(Quaternion.q_rotMatToQuat(R_BC)) # q_CB
inertialTranslation = None
inertialRotation = None
writeData = True

opts, args = getopt.getopt(sys.argv[1:],'i:j:o:')
for opt, arg in opts:
    if opt == '-i':
        odometryBag = arg
        print(odometryBag)
    if opt == '-j':
        timestampBag = arg
    elif opt == '-o':
        testFolder = arg
        
Utils.createFolderIfMissing(testFolder)

# Load Odometry Data
td.addLabelingIncremental('pos',3)
td.addLabelingIncremental('att',4)
td.addLabelingIncremental('vel',3)
td.reInit()
RosDataAcquisition.rosBagLoadOdometry(odometryBag, odometryTopic, td, 'pos','att','vel',None,None,None,None,None)

# Load Timestamps into new td
td_aligned.addLabelingIncremental('pos',3)
td_aligned.addLabelingIncremental('att',4)
td_aligned.reInit()
RosDataAcquisition.rosBagLoadTimestampsOnly(timestampBag,timestampTopic,td_aligned)

# Interpolate Data
td.interpolateColumns(td_aligned, 'pos', 'pos')
td.interpolateQuaternion(td_aligned, 'att', 'att')

ind, = np.nonzero(np.fabs(np.diff(td.col(1))) < 1e-10)
print(ind)
ind, = np.nonzero(np.fabs(np.diff(td_aligned.col(1))) < 1e-10)
print(ind)

# Plotting
plotterPos = Plotter(-1, [3,1],'Position',['','','time[s]'],['x[m]','y[m]','z[m]'],150000)
plotterAtt = Plotter(-1, [4,1],'Attitude',['','','','time[s]'],['w','x','y','z'],150000)
plotterPos.addDataToSubplotMultiple(td, 'pos', [1,2,3], ['b','b','b'], ['','',''])
plotterAtt.addDataToSubplotMultiple(td, 'att', [1,2,3,4], ['b','b','b','b'], ['','','',''])
plotterPos.addDataToSubplotMultiple(td_aligned, 'pos', [1,2,3], ['g','g','g'], ['','',''])
plotterAtt.addDataToSubplotMultiple(td_aligned, 'att', [1,2,3,4], ['g','g','g','g'], ['','','',''])

plotterVel = Plotter(-1, [3,1],'Velocity',['','','time[s]'],['x[m]','y[m]','z[m]'],150000)
plotterVel.addDataToSubplotMultiple(td, 'vel', [1,2,3], ['b','b','b'], ['','',''])

# Apply body and inertial 
if bodyTranslation != None and bodyRotation != None:
    td_aligned.applyBodyTransform('pos', 'att', bodyTranslation, bodyRotation)
if inertialTranslation != None and inertialRotation != None:
    td_aligned.applyBodyTransform('pos', 'att', inertialTranslation, inertialRotation)
plotterPos.addDataToSubplotMultiple(td_aligned, 'pos', [1,2,3], ['r','r','r'], ['','',''])
plotterAtt.addDataToSubplotMultiple(td_aligned, 'att', [1,2,3,4], ['r','r','r','r'], ['','','',''])

ind, = np.nonzero(np.fabs(np.diff(td_aligned.col(1))) < 1e-10)
print(ind)

if writeData:
    td_aligned.writeColsToSingleFiles(testFolder + '/poseOut', 'pose', td_aligned.getColIDs('pos') + td_aligned.getColIDs('att'), ' ')

raw_input("Press Enter to continue...")
