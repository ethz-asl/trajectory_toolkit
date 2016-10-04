# Imports
import os, sys, inspect
import Quaternion
import RosDataAcquisition
import CsvDataAcquisition
import numpy as np
from pylab import *
import matplotlib.pyplot as plt
import rosbag as rb
import Utils

class VIEvaluator:
    bag = ''
    odomTopic = ''
    pclTopic = ''
    extrinsicsTopic = ''
    biasesTopic = ''
    gtFile = ''
    gtTopic = ''
    startcut = 0
    endcut = 0
    MrMV = None
    qVM = None
    extraTransformPos = None
    extraTransformAtt = None
    alignMode = 0
    doCov = False
    doNFeatures = 0
    doExtrinsics = False
    doBiases = False
    td = None
    tdgt = None
    derMode = 1
    aDer = 3
    bDer = 3
    derModeGT = 0
    aDerGT = 3
    bDerGT = 3
    plotLeutiDistances = []
    leutiSpacing = 0

    def initTimedData(self,td):
        td.clearLabeling()
        td.addLabelingIncremental('pos',3)
        td.addLabelingIncremental('att',4)
        td.addLabelingIncremental('vel',3)
        td.addLabelingIncremental('ror',3)
        td.addLabelingIncremental('ron',1)
        td.addLabelingIncremental('ypr',3)
        if self.doCov:
            td.addLabelingIncremental('posCov',9)
            td.addLabelingIncremental('posSp',3)
            td.addLabelingIncremental('posSm',3)
            td.addLabelingIncremental('attCov',9)
            td.addLabelingIncremental('yprCov',9)
            td.addLabelingIncremental('yprSp',3)
            td.addLabelingIncremental('yprSm',3)
            td.addLabelingIncremental('velCov',9)
            td.addLabelingIncremental('velSp',3)
            td.addLabelingIncremental('velSm',3)
            td.addLabelingIncremental('rorCov',9)
            td.addLabelingIncremental('rorSp',3)
            td.addLabelingIncremental('rorSm',3)
        if self.doExtrinsics:
            td.addLabelingIncremental('extPos',3)
            if self.doCov:
                td.addLabelingIncremental('extPosCov',9)
                td.addLabelingIncremental('extPosSp',3)
                td.addLabelingIncremental('extPosSm',3)
            td.addLabelingIncremental('extAtt',4)
            td.addLabelingIncremental('extYpr',3)
            if self.doCov:
                td.addLabelingIncremental('extAttCov',9)
                td.addLabelingIncremental('extYprCov',9)
                td.addLabelingIncremental('extYprSp',3)
                td.addLabelingIncremental('extYprSm',3)
        if self.doBiases:
            td.addLabelingIncremental('gyb',3)
            if self.doCov:
                td.addLabelingIncremental('gybCov',9)
                td.addLabelingIncremental('gybSp',3)
                td.addLabelingIncremental('gybSm',3)
            td.addLabelingIncremental('acb',3)
            if self.doCov:
                td.addLabelingIncremental('acbCov',9)
                td.addLabelingIncremental('acbSp',3)
                td.addLabelingIncremental('acbSm',3)
        if self.doNFeatures > 0:
            td.addLabelingIncremental('feaPos',3,self.doNFeatures)
            td.addLabelingIncremental('feaCov',9,self.doNFeatures)
            td.addLabelingIncremental('feaIdx',1,self.doNFeatures)
            td.addLabelingIncremental('feaDis',1,self.doNFeatures)
            td.addLabelingIncremental('feaDisCov',1,self.doNFeatures)
        td.reInit()
        self.td = td
    
    def initTimedDataGT(self,td):
        td.clearLabeling()
        td.addLabelingIncremental('pos',3)
        td.addLabelingIncremental('att',4)
        td.addLabelingIncremental('vel',3)
        td.addLabelingIncremental('ror',3)
        td.addLabelingIncremental('ron',1)
        td.addLabelingIncremental('ypr',3)
        td.reInit()
        self.tdgt = td
        
    def acquireData(self):
        if(self.bag.endswith('.bag')):
            if self.doCov:
                RosDataAcquisition.rosBagLoadOdometry(self.bag, self.odomTopic ,self.td,'pos','att','vel','ror','posCov','attCov','velCov','rorCov')
            else:
                RosDataAcquisition.rosBagLoadOdometry(self.bag, self.odomTopic ,self.td,'pos','att','vel','ror')
            if self.pclTopic != '' and self.doNFeatures > 0:
                RosDataAcquisition.rosBagLoadRobocentricPointCloud(self.bag,self.pclTopic,self.td,'feaIdx','feaPos','feaCov','feaDis','feaDisCov')
            if self.extrinsicsTopic != '' and self.doExtrinsics:
                if self.doCov:
                    RosDataAcquisition.rosBagLoadPoseWithCovariance(self.bag, self.extrinsicsTopic ,self.td,'extPos','extAtt','extPosCov','extAttCov')
                else:
                    RosDataAcquisition.rosBagLoadPoseWithCovariance(self.bag, self.extrinsicsTopic ,self.td,'extPos','extAtt')
            if self.biasesTopic != '' and self.doBiases:
                if self.doCov:
                    RosDataAcquisition.rosBagLoadImuWithCovariance(self.bag, self.biasesTopic ,self.td,'gyb','acb','gybCov','acbCov')
                else:
                    RosDataAcquisition.rosBagLoadImuWithCovariance(self.bag, self.biasesTopic ,self.td,'gyb','acb')
        if(self.bag.endswith('data.csv')):
            CsvDataAcquisition.csvLoadTransform(self.bag, 0, 1, 4, self.td, 'pos', 'att')
            self.td.d[:,0] = self.td.d[:,0]*1e-9
            attID = self.td.getColIDs('att')
            q_real = self.td.col(attID[3])
            q_im = -self.td.col(attID[0:3])
            self.td.setCol(q_real,attID[0])
            self.td.setCol(q_im,attID[1:4])
        self.td.applyTimeOffset(-self.td.getFirstTime())
        
    def acquireDataGT(self):
        if(self.gtFile.endswith('.csv')):
            CsvDataAcquisition.csvLoadTransform(self.gtFile, 0, 1, 4, self.tdgt, 'pos', 'att')
        elif(self.gtFile.endswith('.bag')):
            RosDataAcquisition.rosBagLoadTransformStamped(self.gtFile,self.gtTopic,self.tdgt,'pos','att')
        if(self.gtFile.endswith('data.csv')):
            self.tdgt.d[:,0] = self.tdgt.d[:,0]*1e-9
        self.tdgt.cropTimes(self.tdgt.getFirstTime()+self.startcut,self.tdgt.getLastTime()-self.endcut)
        self.tdgt.applyTimeOffset(-self.tdgt.getFirstTime())
        
    def alignTime(self):
        self.tdgt.computeNormOfColumns('ror','ron')
        self.td.computeNormOfColumns('ror','ron')
        self.td.applyTimeOffset(self.tdgt.getFirstTime()-self.td.getFirstTime())
        to = self.td.getTimeOffset('ron',self.tdgt,'ron')
        self.td.applyTimeOffset(-to)
        self.td.cropTimes(self.tdgt.getFirstTime(),self.tdgt.getLastTime())
    
    def alignBodyFrame(self):
        if(self.extraTransformPos != None or self.extraTransformAtt != None):
            if(self.alignMode == 0 or self.alignMode == 2):
                self.tdgt.applyBodyTransformFull('pos', 'att','vel', 'ror', self.extraTransformPos, self.extraTransformAtt)
            if(self.alignMode == 1 or self.alignMode == 3):
                self.td.applyBodyTransformFull('pos', 'att','vel', 'ror', self.extraTransformPos, self.extraTransformAtt)
                if(self.doCov):
                    self.td.applyBodyTransformToAttCov('attCov', self.extraTransformAtt)
                    if((self.extraTransformPos != np.array([0.0, 0.0, 0.0])).any()):
                        print('Warning: Covariance are not properly mapped if there is a translational component on the Body transform')
        if(self.alignMode == 0):
            B_r_BC_est, qCB_est = self.td.calibrateBodyTransform('vel', 'ror', self.tdgt, 'vel','ror')
            print('Align Body Transform (estimate to GT):')
            print('Quaternion Rotation qCB_est:\tw:' + str(qCB_est[0]) + '\tx:' + str(qCB_est[1]) + '\ty:' + str(qCB_est[2]) + '\tz:' + str(qCB_est[3]))
            print('Translation Vector B_r_BC_est:\tx:' + str(B_r_BC_est[0]) + '\ty:' + str(B_r_BC_est[1]) + '\tz:' + str(B_r_BC_est[2]))
            self.td.applyBodyTransformFull('pos', 'att','vel', 'ror', B_r_BC_est, qCB_est)
            if(self.doCov):
                print('Warning: Covariance are not properly for Body alignment of estimate to GT, please use the other direction')
        if(self.alignMode == 1):
            B_r_BC_est, qCB_est = self.tdgt.calibrateBodyTransform('vel', 'ror', self.td, 'vel','ror')
            print('Align Body Transform (GT to estimate):')
            print('Quaternion Rotation qCB_est:\tw:' + str(qCB_est[0]) + '\tx:' + str(qCB_est[1]) + '\ty:' + str(qCB_est[2]) + '\tz:' + str(qCB_est[3]))
            print('Translation Vector B_r_BC_est:\tx:' + str(B_r_BC_est[0]) + '\ty:' + str(B_r_BC_est[1]) + '\tz:' + str(B_r_BC_est[2]))
            self.tdgt.applyBodyTransformFull('pos', 'att','vel', 'ror', B_r_BC_est, qCB_est)
        if(self.alignMode == 2):
            B_r_BC_est = self.MrMV
            qCB_est = self.qVM
            print('Fixed Body Alignment (estimate to GT):')
            print('Quaternion Rotation qCB_est:\tw:' + str(qCB_est[0]) + '\tx:' + str(qCB_est[1]) + '\ty:' + str(qCB_est[2]) + '\tz:' + str(qCB_est[3]))
            print('Translation Vector B_r_BC_est:\tx:' + str(B_r_BC_est[0]) + '\ty:' + str(B_r_BC_est[1]) + '\tz:' + str(B_r_BC_est[2]))
            self.td.applyBodyTransformFull('pos', 'att','vel', 'ror', B_r_BC_est, qCB_est)
            if(self.doCov):
                print('Warning: Covariance are not properly for Body alignment of estimate to GT, please use the other direction')
            if(self.extraTransformPos != None or self.extraTransformAtt != None):
                self.td.applyBodyTransformFull('pos', 'att','vel', 'ror', self.extraTransformPos, self.extraTransformAtt)
                if(self.doCov):
                    self.td.applyBodyTransformToAttCov('attCov', self.extraTransformAtt)
                    if(self.extraTransformPos != np.array([0.0, 0.0, 0.0])):
                        print('Warning: Covariance are not properly mapped if there is a translational component on the Body transform')
        if(self.alignMode == 3):
            B_r_BC_est = -Quaternion.q_rotate(self.qVM, self.MrMV)
            qCB_est = Quaternion.q_inverse(self.qVM)
            print('Fixed Body Alignment (GT to estimate):')
            print('Quaternion Rotation qCB_est:\tw:' + str(qCB_est[0]) + '\tx:' + str(qCB_est[1]) + '\ty:' + str(qCB_est[2]) + '\tz:' + str(qCB_est[3]))
            print('Translation Vector B_r_BC_est:\tx:' + str(B_r_BC_est[0]) + '\ty:' + str(B_r_BC_est[1]) + '\tz:' + str(B_r_BC_est[2]))
            self.tdgt.applyBodyTransformFull('pos', 'att','vel', 'ror', B_r_BC_est, qCB_est)
            if(self.extraTransformPos != None or self.extraTransformAtt != None):
                self.tdgt.applyBodyTransformFull('pos', 'att','vel', 'ror', self.extraTransformPos, self.extraTransformAtt)
    
    def alignInertialFrame(self, calIDs=[0,1,2,3,4,5,6]):
        if(self.alignMode == 0 or self.alignMode == 2):
            J_r_JI_est, qIJ_est = self.td.calibrateInertialTransform('pos', 'att', self.tdgt, 'pos','att', np.array([0.0,0.0,0.0]), np.array([1.0,0.0,0.0,0.0]), calIDs)
            print('Align Inertial Transform (estimate to GT):')
            print('Quaternion Rotation qIJ_est:\tw:' + str(qIJ_est[0]) + '\tx:' + str(qIJ_est[1]) + '\ty:' + str(qIJ_est[2]) + '\tz:' + str(qIJ_est[3]))
            print('Translation Vector J_r_JI_est:\tx:' + str(J_r_JI_est[0]) + '\ty:' + str(J_r_JI_est[1]) + '\tz:' + str(J_r_JI_est[2]))
            self.td.applyInertialTransform('pos', 'att',J_r_JI_est,qIJ_est)
        if(self.alignMode == 1 or self.alignMode == 3):
            J_r_JI_est, qIJ_est = self.tdgt.calibrateInertialTransform('pos', 'att', self.td, 'pos','att', np.array([0.0,0.0,0.0]), np.array([1.0,0.0,0.0,0.0]), calIDs)
            print('Align Inertial Transform (GT to estimate):')
            print('Quaternion Rotation qIJ_est:\tw:' + str(qIJ_est[0]) + '\tx:' + str(qIJ_est[1]) + '\ty:' + str(qIJ_est[2]) + '\tz:' + str(qIJ_est[3]))
            print('Translation Vector J_r_JI_est:\tx:' + str(J_r_JI_est[0]) + '\ty:' + str(J_r_JI_est[1]) + '\tz:' + str(J_r_JI_est[2]))
            self.tdgt.applyInertialTransform('pos', 'att',J_r_JI_est,qIJ_est)
    
    def getAllDerivatives(self):
        if(self.derMode == 0):
            self.td.computeRatesFromPose('pos', 'att', 'vel', 'ror', self.aDer, self.bDer)
        if(self.derMode == 2):
            self.td.transformRatesFromWorldToBody('att', 'vel', 'ror')
        if(self.derModeGT == 0):
            self.tdgt.computeRatesFromPose('pos', 'att', 'vel', 'ror', self.aDerGT, self.bDerGT)
        if(self.derModeGT == 2):
            self.tdgt.transformRatesFromWorldToBody('att', 'vel', 'ror')
        self.td.computeNormOfColumns('ror','ron')
        self.tdgt.computeNormOfColumns('ror','ron')
    
    def getYpr(self):
        self.td.quaternionToYpr('att','ypr')
        if self.doCov:
            self.td.quaternionToYprCov('att','attCov','yprCov')
        if self.doExtrinsics:
            self.td.quaternionToYpr('extAtt','extYpr')
            if self.doCov:
                self.td.quaternionToYprCov('extAtt','extAttCov','extYprCov')
        if(self.gtFile != ''):
            self.tdgt.quaternionToYpr('att','ypr')
    
    def evaluateSigmaBounds(self, factor = 3):
        if self.doCov:
            self.td.computeSigmaBounds('pos','posCov','posSp','posSm',factor)
            self.td.computeSigmaBounds('vel','velCov','velSp','velSm',factor)
            self.td.computeSigmaBounds('ror','rorCov','rorSp','rorSm',factor)
            self.td.computeSigmaBounds('ypr','yprCov','yprSp','yprSm',factor)
            if self.doExtrinsics:
                self.td.computeSigmaBounds('extPos','extPosCov','extPosSp','extPosSm',factor)
                self.td.computeSigmaBounds('extYpr','extYprCov','extYprSp','extYprSm',factor)
            if self.doBiases:
                self.td.computeSigmaBounds('gyb','gybCov','gybSp','gybSm',factor)
                self.td.computeSigmaBounds('acb','acbCov','acbSp','acbSm',factor)
    
    def doLeutiEvaluation(self, figureId = -1):
        if len(self.plotLeutiDistances) > 0:
            plt.ion()            
            plt.show(block=False)
            if(figureId == -1):
                figure()
            elif(figureId >= 0):
                figure(figureId)
            spacings = self.plotLeutiDistances
            if self.leutiSpacing > 0:
                spacings = np.ones(len(self.plotLeutiDistances))*self.leutiSpacing
            leutiOutputPos, leutiOutputAtt, leutiOutputYaw, leutiOutputIncl = self.td.computeLeutiScore('pos', 'att', 'vel', self.tdgt, 'pos', 'att', self.plotLeutiDistances, spacings, 0.0)
            p = np.arange(len(self.plotLeutiDistances))
            if(figureId >= -1):
                boxplot(leutiOutputPos, positions = p, widths = 0.8)
                ax = axes()
                ax.set_xticklabels(self.plotLeutiDistances)
                ax.set_xlabel('Travelled Distance [m]')
                ax.set_ylabel('Position Error [m]')
            
            med = np.zeros(len(leutiOutputPos))
            for i in range(len(leutiOutputPos)):
                med[i] = median(leutiOutputPos[i])
            s = np.array(self.plotLeutiDistances)**0.5
            score = s.dot(med)/s.dot(s)
            fit = score*s
            if(figureId >= -1):
                plot(p,fit)
                title('Error Plot for Position, score = ' + str(score))
            
            return [leutiOutputPos, leutiOutputAtt, leutiOutputYaw, leutiOutputIncl, score]
    
    def doFeatureDepthEvaluation(self, figureId = -1, startTime = 25.0 ,plotFeaTimeEnd = 3.0, startAverage = 1.0, frequency = 20.0):
        if self.doNFeatures > 0 and figureId >= -1:
            plt.ion()            
            plt.show(block=False)
            if(figureId == -1):
                figure()
            elif(figureId >= 0):
                figure(figureId)
            x = [[]]
            y = [[]]
            cov = [[]]
            feaIdxID = self.td.getColIDs('feaIdx')
            feaPosID = self.td.getColIDs('feaPos')
            feaCovID = self.td.getColIDs('feaCov')
            for j in np.arange(self.doNFeatures):
                newFea = self.td.col('pos') + Quaternion.q_rotate(Quaternion.q_inverse(self.td.col('att')),self.td.col(feaPosID[j]))
                self.td.applyRotationToCov(feaCovID[j], 'att', True)
                for i in np.arange(0,3):
                    self.td.setCol(newFea[:,i],feaPosID[j][i])
                   
                lastStart = 0.0
                lastID = -1
                startID = 0
                for i in np.arange(self.td.length()):
                    if(self.td.d[i,self.td.timeID] > startTime):
                        if(self.td.d[i,feaIdxID[j]] < 0.0 or self.td.d[i,feaIdxID[j]] != lastID):
                            if len(x[-1]) > 0:
                                x.append([])
                                y.append([])
                                cov.append([])
                        if(self.td.d[i,feaIdxID[j]] >= 0.0):
                            if len(x[-1]) == 0:
                                lastStart = self.td.d[i,self.td.timeID]
                                lastID = self.td.d[i,feaIdxID[j]]
                                startID = i
                            x[-1].append(self.td.d[i,self.td.timeID]-lastStart)
                            y[-1].append(self.td.d[i,feaPosID[j][2]])
                            cov[-1].append(sqrt(self.td.d[i,feaCovID[j][8]]))
               
            axis = subplot(2,1,1)
            for i in np.arange(len(x)):
                plot(x[i],y[i],color=Utils.colors['gray'])
               
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
            print("Final standard deviation: " + str(stds[-1]))
            plot(np.arange(plotFeaTimeEnd*frequency)/frequency,np.ones(plotFeaTimeEnd*frequency)*average, color=Utils.colors['blue'], lw=3, label='Estimated groundtruth')
            plot(np.arange(plotFeaTimeEnd*frequency)/frequency,average+np.array(means),color=Utils.colors['red'], lw=3, label='Average')
            plot(np.arange(plotFeaTimeEnd*frequency)/frequency,average+np.array(means)+np.array(stds),'--',color=Utils.colors['red'], lw=3, label='Measured standard deviation')
            plot(np.arange(plotFeaTimeEnd*frequency)/frequency,average+np.array(means)-np.array(stds),'--',color=Utils.colors['red'], lw=3)
            plot(np.arange(plotFeaTimeEnd*frequency)/frequency,average+np.array(means)+np.array(meanCovs),'--',color=Utils.colors['green'], lw=3, label='Estimated standard deviation')
            plot(np.arange(plotFeaTimeEnd*frequency)/frequency,average+np.array(means)-np.array(meanCovs),'--',color=Utils.colors['green'], lw=3)
            plt.axis([0, plotFeaTimeEnd, -4.0, -1.0])
            plt.legend(loc=4)
            axis.set_ylabel('Landmark height [m]')
            plt.title('Convergence of Landmark Distance')
               
            axis = subplot(2,1,2)
            line_th = plot([0, plotFeaTimeEnd], [sqrt(6.64), sqrt(6.64)],'--',color=Utils.colors['green'], lw=3, label='1\% threshold')
            for i in np.arange(len(x)):
                plot(x[i],np.abs(np.array(y[i])-average)/np.array(cov[i]),color=Utils.colors['gray'])
            plt.axis([0, plotFeaTimeEnd, 0, 5])
            plt.legend()
            axis.set_ylabel('Normalized error [1]')
            axis.set_xlabel('Time [s]')
