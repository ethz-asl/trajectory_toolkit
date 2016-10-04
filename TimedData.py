import numpy as np
import Quaternion
import time
from __builtin__ import classmethod
import math
import Utils
from numpy import arange
from locale import str
import pickle

class TimedData:
    # Data numpy array
    d = 0
    # Number of columns of d / Number of data properties (e.g. 8 for t(1), pos(3), quat(4))
    Nc = 0
    # Index of last rows of d that has an entry
    last = 0
    timeID = 0
    labeling = {}
    
    def __init__(self, Nc = 1):
        self.Nc = Nc; 
        self.last = -1;
        self.d = np.empty([10,Nc]);
        self.labeling = {}
    
    def clearLabeling(self):
        self.labeling = {}
        
    def addLabeling(self,label,colIDs):
        self.labeling[label] = colIDs
        
    def addLabelingIncremental(self,newLabel,numCol,n = 1):
        if(n == 1):
            self.addLabeling(newLabel,range(self.Nc,self.Nc+numCol))
        else:
            colIDs = []
            for i in range(n):
                colIDs.append(range(self.Nc+numCol*i,self.Nc+numCol*(i+1)))
            self.addLabeling(newLabel,colIDs)
        self.Nc = self.Nc + numCol*n
    
    def getColIDs(self,col):
        if(isinstance(col, basestring)):
            if self.labeling.has_key(col):
                return self.labeling[col]
            else:
                print('WARNING: no such column as ' + col + '!')
                return None
        return col
        
    def reInit(self):
        self.last = -1
        self.d = np.empty([10,self.Nc])
    
    def initEmptyFromTimes(self, times): #TESTED
        self.last = np.shape(times)[0]-1
        self.d = np.zeros([np.shape(times)[0], self.Nc])
        self.d[:self.end(),self.timeID] = times;
    
    def cropTimes(self, t0, t1):
        indices, = np.nonzero(np.logical_and(self.getTime() >= t0,self.getTime() <= t1))
        self.d = np.take(self.D(), indices, axis=0)
        self.last = np.size(indices)-1
    
    def setColumnToSine(self, col, amplitude, frequency, phaseShift):
        colID = self.getColIDs(col)
        if(Utils.getLen(colID) > 1):
            print('WARNING: setColumnToSine only implement for single column!')
        else:
            self.setCol(amplitude * np.sin(2 * np.pi * frequency * self.getTime() + phaseShift), colID)
    
    def setCol(self, data, col): #TESTED
        colIDs = self.getColIDs(col)
        if(np.shape(data)[0] == self.end()):
            if(np.all(np.less(colIDs, self.Nc))):
                self.d[:self.end(),colIDs] = data.reshape(np.shape(self.d[:self.end(),colIDs]));
            else:
                print('WARNING: Did not set columns! No column with that ID!')
        else:
            print('WARNING: Did not set columns! Wrong data size!');
    
    def setRow(self, data, rowIDs): #TESTED
        if(np.shape(data)[0] == self.Nc):
            if(np.all(np.less(rowIDs, self.end()))):
                self.d[rowIDs,:] = data;
            else:
                print('WARNING: Did not set row! No row with that ID!')
        else:
            print('WARNING: Did not set row! Wrong data size!');
            
    def setBlock(self, data, rowID, col):
        colIDs = self.getColIDs(col)
        if(np.shape(data)[0] <= (self.end()-rowID) ):
            if(np.shape(data)[1] <= (self.Nc-colIDs) ): 
                self.d[rowID:rowID+np.shape(data)[0],colIDs:colIDs+np.shape(data)[1]] = data;
            else:
                print('WARNING: Did not set block! Columns exceed matrix size!')
        else:
            print('WARNING: Did not set block! Rows exceed matrix size!');
    
    # These getter are used to access the "meaningful" data
    def D(self): #TESTED
        return self.d[:self.end(),];
    
    def col(self, col): #TESTED
        colIDs = self.getColIDs(col)
        # Check colIDs validity
        if(np.any(np.greater(colIDs, np.shape(self.d)[1]-1))):
            print('WARNING: You requested an invalid colIDs. Returning Zeros!');
            print(colIDs)
            return np.zeros([self.end(), Utils.getLen(colIDs)]); 
        # Return data
        return self.d[0:self.end(), colIDs];

    def row(self, rowIDs): #TESTED
        # Check rowID validity
        if(np.any(np.greater(rowIDs, self.last))):
            print('WARNING: You requested an invalid rowID. Returning Zeros!');
            print(rowIDs)
            return np.zeros([Utils.getLen(rowIDs), self.Nc]); 
        # Return data
        return self.d[rowIDs,:];     
   
    def getTime(self):
        return self.col(self.timeID)
   
    def getLastTime(self):
        return self.col(self.timeID)[-1]
    
    def getFirstTime(self):
        return self.col(self.timeID)[0]
    
    def getSubTimedData(self,col = None,rowIDs = None):
        if(col == None):
            col = range(1,self.Nc)
        colIDs = self.getColIDs(col)
        if(rowIDs == None):
            rowIDs = range(0,self.length())
        newTD = TimedData(Utils.getLen(colIDs))
        newTD.initEmptyFromTimes(self.d[rowIDs,0])
        newTD.setCol(self.d[rowIDs,colIDs], range(Utils.getLen(colIDs)))
   
    # Append an entry, resize array
    def append(self): #TESTED
        if np.shape(self.d)[0] == self.end():
            self.d = np.resize(self.d, (2*np.shape(self.d)[0], self.Nc));   
        self.last += 1;

    def end(self): #TESTED
        return (self.last + 1);
    
    def length(self): #TESTED
        return (self.last + 1);
    
    # Math functions
    def computeNormOfColumns(self, col, normID):
        colIDs = self.getColIDs(col)
        self.setCol(Utils.norm(self.col(colIDs)), normID)

    def computeVectorNDerivative(self, colIn, colOut, a=1, b=0): #TESTED
        colInIDs = self.getColIDs(colIn)
        colOutIDs = self.getColIDs(colOut)
        if(Utils.getLen(colInIDs)==Utils.getLen(colOutIDs)):
            dt = self.getTime()[a+b:self.end()]-self.getTime()[0:self.end()-(a+b)]
            for i in xrange(0,Utils.getLen(colInIDs)):
                dp = self.col(colInIDs[i])[a+b:self.end()]-self.col(colInIDs[i])[0:self.end()-(a+b)]
                self.d[0:a,colOutIDs[i]].fill(0)
                self.d[self.end()-b:self.end(),colOutIDs[i]].fill(0)
                self.d[a:self.end()-b,colOutIDs[i]] = np.divide(dp,dt);
        else:
            print('WARNING: Compute N derivative failed! ColIDs did not match in size.')
    
    def computeVelocitiesInBodyFrameFromPostionInWorldFrame(self, colPos, colVel, colAtt, a=1, b=0):
        colPosIDs = self.getColIDs(colPos)
        colVelIDs = self.getColIDs(colVel)
        colAttIDs = self.getColIDs(colAtt)
        self.computeVectorNDerivative(colPosIDs, colVelIDs, a, b)
        self.setCol(Quaternion.q_rotate(self.col(colAttIDs), self.col(colVelIDs)),colVelIDs);
    
    def computeRotationalRateFromAttitude(self, colAtt, colRor, a=1, b=0):
        colAttIDs = self.getColIDs(colAtt)
        colRorIDs = self.getColIDs(colRor)
        dv = Quaternion.q_boxMinus(self.d[a+b:self.end(),colAttIDs],self.d[0:self.end()-(a+b),colAttIDs])
        dt = self.getTime()[a+b:self.end()]-self.getTime()[0:self.end()-(a+b)]
        for i in np.arange(0,3):
            self.d[0:a,colRorIDs[i]].fill(0)
            self.d[self.end()-b:self.end(),colRorIDs[i]].fill(0)
            self.d[a:self.end()-b,colRorIDs[i]] = np.divide(dv[:,i],dt);
    
    def computeRatesFromPose(self, colPos, colAtt, colVel, colRor, a=1, b=0):
        colPosIDs = self.getColIDs(colPos)
        colAttIDs = self.getColIDs(colAtt)
        colVelIDs = self.getColIDs(colVel)
        colRorIDs = self.getColIDs(colRor)
        self.computeVelocitiesInBodyFrameFromPostionInWorldFrame(colPosIDs,colVelIDs,colAttIDs,a,b)
        self.computeRotationalRateFromAttitude(colAttIDs,colRorIDs,a,b)
        
    def transformRatesFromWorldToBody(self, colAtt, colVel, colRor):
        colAttIDs = self.getColIDs(colAtt)
        colVelIDs = self.getColIDs(colVel)
        colRorIDs = self.getColIDs(colRor)
        self.setCol(Quaternion.q_rotate(self.col(colAttIDs), self.col(colRorIDs)),colRorIDs)
        self.setCol(Quaternion.q_rotate(self.col(colAttIDs), self.col(colVelIDs)),colVelIDs)
        
    def interpolateColumns(self, other, colIn, colOut=None): #TESTED
        # Allow interpolating in to other columns / default is into same column
        if colOut is None:
            colOut = colIn
        colInIDs = self.getColIDs(colIn)
        if(isinstance(colInIDs, int)):
            colInIDs = [colInIDs]
        colOutIDs = other.getColIDs(colOut)
        if(isinstance(colOutIDs, int)):
            colOutIDs = [colOutIDs]
        # ColIDs must match in size to be interpolated
        if(Utils.getLen(colInIDs)==Utils.getLen(colOutIDs)):
            if(np.all(np.diff(other.getTime()) > 0)):
                for i in range(0,Utils.getLen(colInIDs)):
                    other.setCol(np.interp(other.getTime(), self.getTime(), self.col(colInIDs[i])),colOutIDs[i])
            else:
                print('WARNING: Interpolation failed! Time values must be increasing.')
        else:
            print('WARNING: Interpolation failed! ColIDs did not match in size.')
    
    def interpolateQuaternion(self, other, colIn, colOut=None):
        # All quaternions before self start are set to the first entry
        if colOut is None:
            colOut = colIn
        colInIDs = self.getColIDs(colIn)
        colOutIDs = other.getColIDs(colOut)
        counterOut = 0;
        counter = 0;
        while other.getTime()[counterOut] <= self.getTime()[counter]:
            other.d[counterOut,colOutIDs] = self.d[counter,colInIDs]
            counterOut += 1
            
        # Interpolation
        while(counterOut < other.length()):
            while(self.getTime()[counter] < other.getTime()[counterOut] and counter < self.last):
                counter += 1
            counter
            if (counter != self.last):
                d = (other.getTime()[counterOut]-self.getTime()[counter-1])/(self.getTime()[counter]-self.getTime()[counter-1]);
                other.d[counterOut,colOutIDs] = Quaternion.q_slerp(self.d[counter-1,colInIDs], self.d[counter,colInIDs], d)   
            else:
                # Set quaternion equal to last quaternion constant extrapolation
                other.d[counterOut,colOutIDs] = self.d[counter,colInIDs];
            counterOut +=1
        
    def getTimeOffset(self,colIn, other, colOut=None):
        if colOut is None:
            colOut = colIn
        colInIDs = self.getColIDs(colIn)
        colOutIDs = other.getColIDs(colOut)
        paddingWithRMS = False
        weightingWithOverlapLength = False
        # Make timing calculation
        dtIn = other.getLastTime()-other.getFirstTime()
        dt = self.getLastTime()-self.getFirstTime()
        timeIncrement = min(dt/(self.length()-1), dtIn/(other.length()-1))
        N = math.floor(dt/timeIncrement)+1
        NIn = math.floor(dtIn/timeIncrement)+1
        # Interpolate of trajectories
        td1 = TimedData(2);
        td2 = TimedData(2);
        if paddingWithRMS:
            td1.initEmptyFromTimes(self.getFirstTime()+np.arange(-(NIn-1),N+(NIn-1))*timeIncrement)
        else:
            td1.initEmptyFromTimes(self.getFirstTime()+np.arange(N)*timeIncrement)
        td2.initEmptyFromTimes(other.getFirstTime()+np.arange(NIn)*timeIncrement)
        self.interpolateColumns(td1, colInIDs, 1)
        other.interpolateColumns(td2, colOutIDs, 1)
        # Padding with RMS
        if paddingWithRMS:
            RMS = np.sqrt(np.mean(np.square(self.col(1))))
            td1.setBlock(np.ones([NIn-1,1])*RMS,0,1)
            td1.setBlock(np.ones([NIn-1,1])*RMS,N+(NIn-1),1)        
        # Calc COnvolution
        if paddingWithRMS:
            conv = np.correlate(td1.D()[:,1], td2.D()[:,1], 'valid')
        else:
            conv = np.correlate(td1.D()[:,1], td2.D()[:,1], 'full')
        if weightingWithOverlapLength:
            w = np.minimum(np.minimum(np.arange(1,N+NIn,1),np.ones(N+NIn-1)*NIn),np.arange(N+NIn-1,0,-1))
            conv = conv / w
        # Deviation of the maximum convolution from the middle of the convolution vector  
        n = np.argmax(conv) - NIn + 1
        return timeIncrement*n + self.getFirstTime() - other.getFirstTime()
    
    def applyTimeOffset(self,to):
        self.setCol(self.col(0) + to,0)
        
    def applyBodyTransform(self, pos, att, translation, rotation):
        posID = self.getColIDs(pos)
        attID = self.getColIDs(att)
        if translation == None:
            translation = np.array([0.0, 0.0, 0.0])
        if rotation == None:
            rotation = np.array([1.0, 0, 0, 0])
        newTranslation = self.col(posID) \
                         + Quaternion.q_rotate(Quaternion.q_inverse(self.col(attID)),
                                               np.kron(np.ones([self.length(),1]),translation))
        newRotation = Quaternion.q_mult(np.kron(np.ones([self.length(),1]),rotation),
                                        self.col(attID))
        for i in np.arange(0,3):
            self.setCol(newTranslation[:,i],posID[i])
        for i in np.arange(0,4):
            self.setCol(newRotation[:,i],attID[i])
    
    def applyBodyTransformToTwist(self, vel, ror, translation, rotation):
        velID = self.getColIDs(vel)
        rorID = self.getColIDs(ror)
        newVel = Quaternion.q_rotate(np.kron(np.ones([self.length(),1]),rotation),
                                     self.col(velID) \
                                     + np.cross(self.col(rorID),np.kron(np.ones([self.length(),1]),translation)))
        newRor = Quaternion.q_rotate(np.kron(np.ones([self.length(),1]),rotation),
                                     self.col(rorID))
        for i in np.arange(0,3):
            self.setCol(newVel[:,i],velID[i])
            self.setCol(newRor[:,i],rorID[i])
    
    def applyBodyTransformFull(self, pos, att, vel, ror, translation, rotation):
        self.applyBodyTransform(pos, att, translation, rotation)
        self.applyBodyTransformToTwist(vel, ror, translation, rotation)
        
    def applyInertialTransform(self, pos, att, translation, rotation):
        posID = self.getColIDs(pos)
        attID = self.getColIDs(att)
        newTranslation = np.kron(np.ones([self.length(),1]),translation) \
                         + Quaternion.q_rotate(np.kron(np.ones([self.length(),1]),Quaternion.q_inverse(rotation)),
                                               self.col(posID))
        newRotation = Quaternion.q_mult(self.col(attID),
                                        np.kron(np.ones([self.length(),1]),rotation))
        for i in np.arange(0,3):
            self.setCol(newTranslation[:,i],posID[i])
        for i in np.arange(0,4):
            self.setCol(newRotation[:,i],attID[i])
        
    def invertRotation(self, att):
        attID = self.getColIDs(att)
        newRotation = Quaternion.q_inverse(self.col(attID))
        for i in np.arange(0,4):
            self.setCol(newRotation[:,i],attID[i])
        
    def invertTransform(self, pos, att):
        posID = self.getColIDs(pos)
        attID = self.getColIDs(att)
        newTranslation = -Quaternion.q_rotate(self.col(attID),self.col(posID))
        for i in np.arange(0,3):
            self.setCol(newTranslation[:,i],posID[i])
        self.invertRotation(attID)
    
    def calibrateBodyTransform(self, vel1, ror1, other, vel2, ror2):
        velID1 = self.getColIDs(vel1)
        rorID1 = self.getColIDs(ror1)
        velID2 = other.getColIDs(vel2)
        rorID2 = other.getColIDs(ror2)
        # Make timing calculation
        dt1 = self.getLastTime()-self.getFirstTime()
        dt2 = other.getLastTime()-other.getFirstTime()
        first = max(self.getFirstTime(),other.getFirstTime())
        last = min(self.getLastTime(),other.getLastTime())
        timeIncrement = min(dt1/(self.length()-1), dt2/(other.length()-1))
        td1 = TimedData(7);
        td2 = TimedData(7);
        td1.initEmptyFromTimes(np.arange(first,last,timeIncrement))
        td2.initEmptyFromTimes(np.arange(first,last,timeIncrement))
        self.interpolateColumns(td1, velID1, [1,2,3])
        self.interpolateColumns(td1, rorID1, [4,5,6])
        other.interpolateColumns(td2, velID2, [1,2,3])
        other.interpolateColumns(td2, rorID2, [4,5,6])
        AA = np.zeros([4,4])
        # TODO: Speed up by removing for loop
        for i in np.arange(0,td1.length()):
            q1 = np.zeros(4)
            q2 = np.zeros(4)
            q1[1:4] = td1.D()[i,4:7];
            q2[1:4] = td2.D()[i,4:7];
            A = Quaternion.q_Rmat(q1)-Quaternion.q_Lmat(q2)
            AA += A.T.dot(A)
        w, v = np.linalg.eigh(AA)
        rotation = v[:,0]
        
        # Find transformation
        A = np.zeros([3*td1.length(),3])
        b = np.zeros([3*td1.length()])
        for i in np.arange(0,td1.length()):
            A[3*i:3*i+3,] = np.resize(Utils.skew(td1.D()[i,4:7]),(3,3))
            b[3*i:3*i+3] = Quaternion.q_rotate(Quaternion.q_inverse(rotation), td2.D()[i,1:4])-td1.D()[i,1:4]
        translation = np.linalg.lstsq(A, b)[0]
        
        return translation, rotation
        
    def calibrateInertialTransform(self, pos1, att1, other, pos2, att2, B_r_BC, q_CB, calIDs=[0]):      
        posID1 = self.getColIDs(pos1)
        attID1 = self.getColIDs(att1)
        posID2 = other.getColIDs(pos2)
        attID2 = other.getColIDs(att2)
        # Make timing calculation 
        dt1 = self.getLastTime()-self.getFirstTime()
        dt2 = other.getLastTime()-other.getFirstTime()
        first = max(self.getFirstTime(),other.getFirstTime())
        last = min(self.getLastTime(),other.getLastTime())
        timeIncrement = min(dt1/(self.length()-1), dt2/(other.length()-1))
        td1 = TimedData(8);
        td2 = TimedData(8);
        td1.initEmptyFromTimes(np.arange(first,last,timeIncrement))
        td2.initEmptyFromTimes(np.arange(first,last,timeIncrement))
        self.interpolateColumns(td1, posID1, [1,2,3])
        other.interpolateColumns(td2, posID2, [1,2,3])
        self.interpolateQuaternion(td1, attID1, [4,5,6,7])
        other.interpolateQuaternion(td2, attID2, [4,5,6,7])
        
        if(not calIDs):
            calIDs = range(td1.length())
        newIDs = np.arange(0,Utils.getLen(calIDs))
        q_CB_vec = np.kron(np.ones([Utils.getLen(calIDs),1]),q_CB)
        q_JC_vec = Quaternion.q_inverse(td2.D()[newIDs,4:8])
        q_BI_vec = td1.D()[newIDs,4:8]
        B_r_BC_vec = np.kron(np.ones([Utils.getLen(calIDs),1]),B_r_BC)
        J_r_JC = td2.D()[newIDs,1:4];
        J_r_BC = Quaternion.q_rotate(Quaternion.q_mult(q_JC_vec,q_CB_vec), B_r_BC_vec)
        J_r_IB = Quaternion.q_rotate(Quaternion.q_mult(q_JC_vec,Quaternion.q_mult(q_CB_vec,q_BI_vec)),td1.D()[newIDs,1:4])
        
        rotation = Quaternion.q_mean(Quaternion.q_inverse(Quaternion.q_mult(Quaternion.q_mult(q_JC_vec,q_CB_vec),q_BI_vec)))
        translation = np.mean(J_r_JC-J_r_BC-J_r_IB, axis=0)
        return translation.flatten(), rotation.flatten()
    
    def computeSigmaBounds(self, data, cov, plus, minus, factor):   
        dataID = self.getColIDs(data)
        covID = self.getColIDs(cov)
        plusID = self.getColIDs(plus)
        minusID = self.getColIDs(minus)
        if(type(dataID) is int):
            dataID = [dataID]
            covID = [covID]
            covDiag = covID
            plusID = [plusID]
            minusID = [minusID]
        else:
            covDiag = []
            for i in range(0,Utils.getLen(dataID)):
                covDiag.append(covID[i*(Utils.getLen(dataID)+1)])
        for i in range(0,Utils.getLen(dataID)):
            self.setCol(self.col(dataID[i])+factor*self.col(covDiag[i])**(1./2),plusID[i])
            self.setCol(self.col(dataID[i])-factor*self.col(covDiag[i])**(1./2),minusID[i])
            
    def quaternionToYpr(self, att, ypr):
        attIDs = self.getColIDs(att)
        yprIDs = self.getColIDs(ypr)
        self.d[0:self.end(),yprIDs] = Quaternion.q_toYpr(self.col(attIDs))
            
    def quaternionToYprCov(self, att, attCov, yprCov):
        attIDs = self.getColIDs(att)
        attCovIDs = self.getColIDs(attCov)
        yprCovIDs = self.getColIDs(yprCov)
        J = Quaternion.q_toYprJac(self.col(attIDs))
        # Do the multiplication by hand, improve if possible
        for i in xrange(0,self.length()):
            self.d[i,yprCovIDs] = np.resize(np.dot(np.dot(np.resize(J[i,:],(3,3)),np.resize(self.col(attCovIDs)[i,:],(3,3))),np.resize(J[i,:],(3,3)).T),(9))
    
    def quaternionToYprFull(self, att, attCov, ypr, yprCov):
        self.quaternionToYpr(att, ypr)
        self.quaternionToYprCov(att, attCov, yprCov)
            
    def applyBodyTransformToAttCov(self, attCov, rotation):
        attCovIDs = self.getColIDs(attCov)
        R = np.resize(Quaternion.q_toRotMat(rotation),(3,3))
        # Do the multiplication by hand, improve if possible
        for i in xrange(0,self.length()):
            self.d[i,attCovIDs] = np.resize(np.dot(np.dot(R,np.resize(self.d[i,attCovIDs],(3,3))),R.T),(9))
            
    def applyRotationToCov(self, cov, att, doInverse=False):
        covIDs = self.getColIDs(cov)
        attIDs = self.getColIDs(att)
        # Do the multiplication by hand, improve if possible
        for i in xrange(0,self.length()):
            R = np.resize(Quaternion.q_toRotMat(self.d[i,attIDs]),(3,3))
            if(doInverse):
                self.d[i,covIDs] = np.resize(np.dot(np.dot(R.T,np.resize(self.d[i,covIDs],(3,3))),R),(9))
            else:
                self.d[i,covIDs] = np.resize(np.dot(np.dot(R,np.resize(self.d[i,covIDs],(3,3))),R.T),(9))
    
    def computeLeutiScore(self, pos1, att1, vel1, other, pos2, att2, distances, spacings, start):
        posID1 = self.getColIDs(pos1)
        attID1 = self.getColIDs(att1)
        velID1 = self.getColIDs(vel1)
        posID2 = other.getColIDs(pos2)
        attID2 = other.getColIDs(att2)
        output = np.empty((Utils.getLen(distances),6))
        outputPosFull = []
        outputAttFull = []
        outputYawFull = []
        outputInclFull = []
        startIndex = np.nonzero(self.getTime() >= start)[0][0]
        
        for j in np.arange(Utils.getLen(distances)):
            tracks = [[startIndex, 0.0]]
            lastAddedStart = 0.0
            posErrors = []
            attErrors = []
            yawErrors = []
            inclErrors = []
        
            other_interp = TimedData(8)
            other_interp.initEmptyFromTimes(self.getTime())
            other.interpolateColumns(other_interp, posID2, [1,2,3])
            other.interpolateQuaternion(other_interp, attID2, [4,5,6,7])
            
            it = startIndex
            while it+1<self.last:
                # Check for new seeds
                if(lastAddedStart>spacings[j]):
                    tracks.append([it, 0.0])
                    lastAddedStart = 0.0
                # Add travelled distance
                d = np.asscalar(Utils.norm(self.d[it,velID1]*(self.d[it+1,0]-self.d[it,0])))
                lastAddedStart += d
                tracks = [[x[0], x[1]+d] for x in tracks]
                # Check if travelled distance large enough
                while Utils.getLen(tracks) > 0 and tracks[0][1] > distances[j]:
                    pos1 = Quaternion.q_rotate(self.d[tracks[0][0],attID1], self.d[it+1,posID1]-self.d[tracks[0][0],posID1])
                    pos2 = Quaternion.q_rotate(other_interp.d[tracks[0][0],[4,5,6,7]], other_interp.d[it+1,[1,2,3]]-other_interp.d[tracks[0][0],[1,2,3]])
                    att1 = self.d[it+1,attID1]
                    att2 = Quaternion.q_mult(other_interp.d[it+1,[4,5,6,7]],
                                             Quaternion.q_mult(Quaternion.q_inverse(other_interp.d[tracks[0][0],[4,5,6,7]]),
                                             self.d[tracks[0][0],attID1]))
                    posErrors.append(np.asscalar(np.sum((pos2-pos1)**2,axis=-1)))
                    attErrors.append(np.asscalar(np.sum((Quaternion.q_boxMinus(att1,att2))**2,axis=-1)))
                    yawError = Quaternion.q_toYpr(Quaternion.q_mult(Quaternion.q_inverse(att2),att1))[2]
                    yawErrors.append(yawError**2)
                    att2_cor = Quaternion.q_mult(att2,Quaternion.q_exp(np.array([0.0,0.0,yawError])))
                    inclErrors.append(np.asscalar(np.sum((Quaternion.q_boxMinus(att1,att2_cor))**2,axis=-1)))
                    tracks.pop(0)
                it += 1
            
            N = Utils.getLen(posErrors)
            posErrorRMS = (np.sum(posErrors,axis=-1)/N)**(0.5)
            posErrorMedian = np.median(posErrors)**(0.5)
            posErrorStd = np.std(np.array(posErrors)**(0.5))
            attErrorRMS = (np.sum(attErrors,axis=-1)/N)**(0.5)
            attErrorMedian = np.median(attErrors)**(0.5)
            attErrorStd = np.std(np.array(attErrors)**(0.5))
            yawErrorRMS = (np.sum(yawErrors,axis=-1)/N)**(0.5)
            yawErrorMedian = np.median(yawErrors)**(0.5)
            yawErrorStd = np.std(np.array(yawErrors)**(0.5))
            inclErrorRMS = (np.sum(inclErrors,axis=-1)/N)**(0.5)
            inclErrorMedian = np.median(inclErrors)**(0.5)
            inclErrorStd = np.std(np.array(inclErrors)**(0.5))
            print('Evaluated ' + str(N) + ' segments with a travelled distance of ' + str(distances[j]) \
                   + ':\n  posRMS of ' + str(posErrorRMS) + ' (Median: ' + str(posErrorMedian) + ', Std: ' + str(posErrorStd) + ')' \
                   + '\n  attRMS of ' + str(attErrorRMS) + ' (Median: ' + str(attErrorMedian) + ', Std: ' + str(attErrorStd) + ')' \
                   + '\n  yawRMS of ' + str(yawErrorRMS) + ' (Median: ' + str(yawErrorMedian) + ', Std: ' + str(yawErrorStd) + ')' \
                   + '\n  inclRMS of ' + str(inclErrorRMS) + ' (Median: ' + str(inclErrorMedian) + ', Std: ' + str(inclErrorStd) + ')')
            output[j,:] = [posErrorRMS, posErrorMedian, posErrorStd, attErrorRMS, attErrorMedian, attErrorStd]
            outputPosFull.append(np.array(posErrors)**(0.5))
            outputAttFull.append(np.array(attErrors)**(0.5))
            outputYawFull.append(np.array(yawErrors)**(0.5))
            outputInclFull.append(np.array(inclErrors)**(0.5))
        return outputPosFull, outputAttFull, outputYawFull, outputInclFull
    
    def pickleDump(self, filename):
        f = open(filename, 'wb')
        pickle.dump(self, f)
    
    def pickleLoad(self, filename):
        f = open(filename, 'rb')
        self = pickle.load(f)
    
    def writeColsToSingleFiles(self, folder, name, cols, separator = ',', f = '{0:05d}'):
        Utils.createFolderIfMissing(folder)
        extractedCols = self.col(cols)
        for i in range(0,self.length()):
            fileOut = folder + '/' + name + f.format(i) + '.txt'
            data = str(extractedCols[i,0])
            for j in range(1,len(cols)):
                data = data + separator + str(extractedCols[i,j])
            with open(fileOut, 'w') as infoFile:
                infoFile.write(data)
            


