# Imports
import os, sys, inspect
from TimedData import TimedData
from Plotter import Plotter
from VIEvaluator import VIEvaluator
import Quaternion
import numpy as np

plotRon = False
plotAtt = False
plotPos = True
plotVel = True
plotRor = False
plotYpr = True
plotExt = True

td_rovio = TimedData()
td_vicon = TimedData()

rovioEvaluator = VIEvaluator()
rovioEvaluator.bag = '/home/michael/datasets/bloesch_leo/planar_1/rovio/2016-02-02-17-18-09_25_4_6_2_0.bag'
rovioEvaluator.odomTopic = '/rovio/odometry'
rovioEvaluator.pclTopic = '/rovio/pcl'
rovioEvaluator.extrinsicsTopic = '/rovio/extrinsics0'
rovioEvaluator.gtFile = '/home/michael/datasets/bloesch_leo/planar_1/2015-12-22-14-15-42.bag'
rovioEvaluator.gtTopic = '/bluebird/vrpn_client/estimated_transform'
rovioEvaluator.startcut = 0
rovioEvaluator.endcut = 0
rovioEvaluator.doCov = True
rovioEvaluator.doNFeatures = 25
rovioEvaluator.doExtrinsics = False
rovioEvaluator.doBiases = False
rovioEvaluator.alignMode = 1
rovioEvaluator.plotLeutiDistances = []

rovioEvaluator.initTimedData(td_rovio)
rovioEvaluator.initTimedDataGT(td_vicon)
rovioEvaluator.acquireData()
rovioEvaluator.acquireDataGT()
rovioEvaluator.getAllDerivatives()
rovioEvaluator.alignTime()
rovioEvaluator.alignBodyFrame()
rovioEvaluator.alignInertialFrame()
rovioEvaluator.getYpr()
rovioEvaluator.evaluateSigmaBounds()
   
if plotPos: # Position plotting
    plotterPos = Plotter(-1, [3,1],'Position',['','','time[s]'],['x[m]','y[m]','z[m]'],10000)
    if rovioEvaluator.doCov:
        plotterPos.addDataToSubplotMultiple(td_rovio, 'posSm', [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterPos.addDataToSubplotMultiple(td_rovio, 'posSp', [1,2,3], ['r--','r--','r--'], ['','',''])
    plotterPos.addDataToSubplotMultiple(td_rovio, 'pos', [1,2,3], ['r','r','r'], ['','',''])
    plotterPos.addDataToSubplotMultiple(td_vicon, 'pos', [1,2,3], ['b','b','b'], ['','',''])

if plotVel: # Velocity plotting
    plotterVel = Plotter(-1, [3,1],'Robocentric Velocity',['','','time[s]'],['$v_x$[m/s]','$v_y$[m/s]','$v_z$[m/s]'],10000)
    plotterVel.addDataToSubplotMultiple(td_rovio, 'vel', [1,2,3], ['r','r','r'], ['','',''])
    plotterVel.addDataToSubplotMultiple(td_vicon, 'vel', [1,2,3], ['b','b','b'], ['','',''])
    
if plotAtt: # Attitude plotting
    plotterAtt = Plotter(-1, [4,1],'Attitude Quaternion',['','','','time[s]'],['w[1]','x[1]','y[1]','z[1]'],10000)
    plotterAtt.addDataToSubplotMultiple(td_rovio, 'att', [1,2,3,4], ['r','r','r','r'], ['','','',''])
    plotterAtt.addDataToSubplotMultiple(td_vicon, 'att', [1,2,3,4], ['b','b','b','b'], ['','','',''])

if plotYpr: # Yaw-pitch-roll plotting
    plotterYpr = Plotter(-1, [3,1],'Yaw-Pitch-Roll Decomposition',['','','time[s]'],['roll[rad]','pitch[rad]','yaw[rad]'],10000)
    if rovioEvaluator.doCov:
        plotterYpr.addDataToSubplotMultiple(td_rovio, 'yprSm', [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterYpr.addDataToSubplotMultiple(td_rovio, 'yprSp', [1,2,3], ['r--','r--','r--'], ['','',''])
    plotterYpr.addDataToSubplotMultiple(td_rovio, 'ypr', [1,2,3], ['r','r','r'], ['','',''])
    plotterYpr.addDataToSubplotMultiple(td_vicon, 'ypr', [1,2,3], ['b','b','b'], ['','',''])
    
if plotRor: # Rotational rate plotting
    plotterRor = Plotter(-1, [3,1],'Rotational Rate',['','','time[s]'],['$\omega_x$[rad/s]','$\omega_y$[rad/s]','$\omega_z$[rad/s]'],10000)
    plotterRor.addDataToSubplotMultiple(td_rovio, 'ror', [1,2,3], ['r','r','r'], ['','',''])
    plotterRor.addDataToSubplotMultiple(td_vicon, 'ror', [1,2,3], ['b','b','b'], ['','',''])

if plotRon: # Plotting rotational rate norm
    plotterRon = Plotter(-1, [1,1],'Norm of Rotational Rate',['time [s]'],['Rotational Rate Norm [rad/s]'],10000)
    plotterRon.addDataToSubplot(td_rovio, 'ron', 1, 'r', 'rovio rotational rate norm')
    plotterRon.addDataToSubplot(td_vicon, 'ron', 1, 'b', 'vicon rotational rate norm')

if plotExt and rovioEvaluator.doExtrinsics: # Extrinsics Plotting
    plotterExt = Plotter(-1, [3,1],'Extrinsics Translational Part',['','','time[s]'],['x[m]','y[m]','z[m]'],10000)
    if rovioEvaluator.doCov:
        plotterExt.addDataToSubplotMultiple(td_rovio, 'extPosSm', [1,2,3], ['r--','r--','r--'], ['','',''])
        plotterExt.addDataToSubplotMultiple(td_rovio, 'extPosSp', [1,2,3], ['r--','r--','r--'], ['','',''])
    plotterExt.addDataToSubplotMultiple(td_rovio, 'extPos', [1,2,3], ['r','r','r'], ['','',''])

rovioEvaluator.doLeutiEvaluation()
rovioEvaluator.doFeatureDepthEvaluation()

raw_input("Press Enter to continue...")
           
         
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
#         errors.append([])# 
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
