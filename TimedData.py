import numpy as np
import Quaternion
import time
from __builtin__ import classmethod
from termcolor import colored
import math
import Utils

class TimedData:
    # Data numpy array
    d = 0
    # Number of columns of d / Number of data properties (e.g. 8 for t(1), pos(3), quat(4))
    Nc = 0
    # Index of last rows of d that has an entry
    last = 0
    timeID = 0
    
    def __init__(self, Nc):
        self.Nc = Nc; 
        self.last = -1;
        self.d = np.empty([10,Nc]);
    
    def initEmptyFromTimes(self, times): #TESTED
        self.last = np.shape(times)[0]-1
        self.d = np.zeros([np.shape(times)[0], self.Nc])
        self.d[:self.end(),self.timeID] = times;
    
    def setColumnToSine(self, colID, amplitude, frequency, phaseShift):
        self.setCol(amplitude * np.sin(2 * np.pi * frequency * self.getTime() + phaseShift), colID)
    
    def setCol(self, data, colID): #TESTED
        if(np.shape(data)[0] == self.end()):
            if(colID < self.Nc):
                self.d[:self.end(),colID] = data;
            else:
                print(colored('WARNING: Did not set column! No column with that ID!','yellow'))
        else:
            print(colored('WARNING: Did not set column! Wrong data size!','yellow'));
    
    def setCols(self, data, colIDs): #TESTED
        if(np.shape(data)[0] == self.end()):
            if(np.all(colIDs) < self.Nc):
                self.d[:self.end(),colIDs] = data;
            else:
                print(colored('WARNING: Did not set columns! No column with that ID!','yellow'))
        else:
            print(colored('WARNING: Did not set columns! Wrong data size!','yellow'));
    
    def setRow(self, data, rowID): #TESTED
        if(np.shape(data)[0] == self.Nc):
            if(rowID < self.end()):
                self.d[rowID,:] = data;
            else:
                print(colored('WARNING: Did not set row! No row with that ID!','yellow'))
        else:
            print(colored('WARNING: Did not set row! Wrong data size!','yellow'));
            
    def setBlock(self, data, rowID, colID):
        if(np.shape(data)[0] <= (self.end()-rowID) ):
            if(np.shape(data)[1] <= (self.Nc-colID) ): 
                self.d[rowID:rowID+np.shape(data)[0],colID:colID+np.shape(data)[1]] = data;
            else:
                print(colored('WARNING: Did not set block! Columns exceed matrix size!','yellow'))
        else:
            print(colored('WARNING: Did not set block! Rows exceed matrix size!','yellow'));
    
    # These getter are used to access the "meaningful" data
    def D(self): #TESTED
        return self.d[:self.end(),];
    
    def col(self, colID): #TESTED
        # Check colID validity
        if (colID > (np.shape(self.d)[1]-1) ):
            print(colored('WARNING: You requested an invalid colID = '+str(colID) + '. Returning Zeros!','yellow'));
            return np.zeros([self.end()]); 
        # Return data
        return self.d[0:self.end(), colID];
    
    def cols(self, colIDs): #TESTED
        # Check colIDs validity
        if (max(colIDs) > (np.shape(self.d)[1]-1) ):
            print(colored('WARNING: You requested an invalid colIDs = '+str(colIDs) + '. Returning Zeros!','yellow'));
            return np.zeros([self.end()]); 
        # Return data
        return self.d[0:self.end(), colIDs];

    def row(self, rowID): #TESTED
        # Check rowID validity
        if(rowID > self.last):
            print(colored('WARNING: You requested an invalid rowID = '+str(rowID) + '. Returning Zeros!','yellow'));
            return np.zeros([self.Nc]); 
        # Return data
        return self.d[rowID,:];     
   
    def getTime(self):
        return self.col(self.timeID)
   
    def getLastTime(self):
        return self.col(self.timeID)[-1]
    
    def getFirstTime(self):
        return self.col(self.timeID)[0]
   
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
    def computeNormOfColumns(self, colID, normID):
        self.setCol(Utils.norm(self.cols(colID)), normID)
        
    def computeDerivativeOfColumn(self, dataID, derivativeID): #TESTED
        dp = np.diff(self.col(dataID))
        dt = np.diff(self.getTime())
        self.d[1:self.end(),derivativeID] = np.divide(dp,dt);

    def computeVectorNDerivative(self, inputIDs, outputIDs): #TESTED
        if(len(inputIDs)==len(outputIDs)):
            dt = np.diff(self.getTime())
            for i in xrange(0,len(inputIDs)):
                dp = np.diff(self.col(inputIDs[i]))
                self.d[1:self.end(),outputIDs[i]] = np.divide(dp,dt);
        else:
            print(colored('WARNING: Compute N derivative failed! ColIDs did not match in size.','yellow'))
    
    def computeVelocitiesInBodyFrameFromPostionInWorldFrame(self, posIDs, outputIDs, q_IB):
        self.computeVectorNDerivative(posIDs, outputIDs)
        self.setCols(Quaternion.q_rotate(q_IB, self.cols(outputIDs)),outputIDs);
    
    def computeRotationalRateFromAttitude(self, attitudeID, rotationalrateID):
        dv = Quaternion.q_boxMinus(self.d[1:self.end(),attitudeID:attitudeID+4],self.d[0:self.last,attitudeID:attitudeID+4])
        dt = np.diff(self.getTime())
        for i in np.arange(0,3):
            self.d[1:self.end(),rotationalrateID+i] = np.divide(dv[:,i],dt);
        
    def interpolateColumns(self, other, colIDs2, colIDs1=None): #TESTED
        # Allow interpolating in to other columns / default is into same column
        if colIDs1 is None:
            colIDs1 = colIDs2
        # ColIDs must match in size to be interpolated
        if(len(colIDs2)==len(colIDs1)):
            for i in xrange(0,len(colIDs2)):
                self.interpolateColumn(other, colIDs1[i], colIDs2[i])
        else:
            print(colored('WARNING: Interpolation failed! ColIDs did not match in size.','yellow'))

    def interpolateColumn(self, other, colID2, colID1=None): #TESTED
        # NOTE: Values outside of the timerange of self are set to the first rsp. last value (no extrapolation)
        # Allow interpolating in to other columns / default is into same column
        if colID1 is None:
            colID1 = colID2
        # When times are increasing interpolate using built in numpy interpolation
        if(np.all(np.diff(other.getTime()) > 0)):
            other.setCol(np.interp(other.getTime(), self.getTime(), self.col(colID1)),colID2)
        else:
            print(colored('WARNING: Interpolation failed! Time values must be increasing.','yellow'))
    
    def interpolateQuaternion(self, tdOut, tdOutColID, colID):
        # All quaternions before self start are set to the first entry
        counterOut = 0;
        counter = 0;
        print(self.D())
        while tdOut.getTime()[counterOut] <= self.getTime()[counter]:
            print('Here')
            tdOut.d[counterOut,tdOutColID:tdOutColID+4] = self.d[counter,colID:colID+4]
            counterOut += 1
            
        # Interpolation
        while(counterOut < tdOut.length()):
            while(self.getTime()[counter] < tdOut.getTime()[counterOut] and counter < self.last):
                counter += 1
            counter
            if (counter != self.last):
                d = (tdOut.getTime()[counterOut]-self.getTime()[counter-1])/(self.getTime()[counter]-self.getTime()[counter-1]);
                tdOut.d[counterOut,tdOutColID:tdOutColID+4] = Quaternion.q_slerp(self.d[counter-1,colID:colID+4], self.d[counter,colID:colID+4], d)   
            else:
                # Set quaternion equal to last quaternion constant extrapolation
                tdOut.d[counterOut,tdOutColID:tdOutColID+4] = self.d[counter,colID:colID+4];
            counterOut +=1
        
    def getTimeOffset(self,other, colID1, colID2=None):
        paddingWithRMS = False
        weightingWithOverlapLength = False
        if colID2 is None:
            colID2 = colID1
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
        self.interpolateColumn(td1, 1, colID1)
        other.interpolateColumn(td2, 1, colID2)
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
        
    def applyBodyTransform(self, translationID, rotationID, translation, rotation):
        newTranslation = self.cols(np.arange(translationID,translationID+3)) \
                         + Quaternion.q_rotate(Quaternion.q_inverse(self.cols(np.arange(rotationID,rotationID+4))),
                                               np.kron(np.ones([self.length(),1]),translation))
        newRotation = Quaternion.q_mult(np.kron(np.ones([self.length(),1]),rotation),
                                        self.cols(np.arange(rotationID,rotationID+4)))
        for i in np.arange(0,3):
            self.setCol(newTranslation[:,i],translationID+i)
        for i in np.arange(0,4):
            self.setCol(newRotation[:,i],rotationID+i)
        
    def applyInertialTransform(self, translationID, rotationID, translation, rotation):
        newTranslation = np.kron(np.ones([self.length(),1]),translation) \
                         + Quaternion.q_rotate(np.kron(np.ones([self.length(),1]),Quaternion.q_inverse(rotation)),
                                               self.cols(np.arange(translationID,translationID+3)))
        newRotation = Quaternion.q_mult(self.cols(np.arange(rotationID,rotationID+4)),
                                        np.kron(np.ones([self.length(),1]),rotation))
        for i in np.arange(0,3):
            self.setCol(newTranslation[:,i],translationID+i)
        for i in np.arange(0,4):
            self.setCol(newRotation[:,i],rotationID+i)
        
    def invertRotation(self, rotationID):
        newRotation = Quaternion.q_inverse(self.cols(np.arange(rotationID,rotationID+4)))
        for i in np.arange(0,4):
            self.setCol(newRotation[:,i],rotationID+i)
        
    def invertTransform(self, translationID, rotationID):
        newTranslation = -Quaternion.q_rotate(self.cols(np.arange(rotationID,rotationID+4)),self.cols(np.arange(translationID,translationID+3)))
        for i in np.arange(0,3):
            self.setCol(newTranslation[:,i],translationID+i)
        self.invertRotation(rotationID)
    
    def calibrateBodyTransform(self, velID1, rorID1, other, velID2, rorID2):
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
        self.interpolateColumns(td1, [velID1+0,velID1+1,velID1+2,rorID1+0,rorID1+1,rorID1+2], [1,2,3,4,5,6])
        other.interpolateColumns(td2, [velID2+0,velID2+1,velID2+2,rorID2+0,rorID2+1,rorID2+2], [1,2,3,4,5,6])
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
            A[3*i:3*i+3,] = Utils.skew(td1.D()[i,4:7])
            b[3*i:3*i+3] = Quaternion.q_rotate(Quaternion.q_inverse(rotation), td2.D()[i,1:4])-td1.D()[i,1:4]
        translation = np.linalg.lstsq(A, b)[0]
        
        return translation, rotation
        
    def calibrateInertialTransform(self, posID1, attID1, other, posID2, attID2, calIDs, q_CB, B_r_BC):        
        # Make timing calculation 
        dt1 = self.getLastTime()-self.getFirstTime()
        dt2 = other.getLastTime()-other.getFirstTime()
        first = max(self.getFirstTime(),other.getFirstTime())
        timeIncrement = min(dt1/(self.length()-1), dt2/(other.length()-1))
        
        # Interpolate 2 entries (min Size, we only need the first entry
        td1 = TimedData(8);
        td2 = TimedData(8);
        ids = first+np.array(calIDs)*timeIncrement;
        print(ids)
        td1.initEmptyFromTimes(ids)
        td2.initEmptyFromTimes(ids)
        self.interpolateColumns(td1, [posID1+0,posID1+1,posID1+2], [1,2,3])
        other.interpolateColumns(td2, [posID2+0,posID2+1,posID2+2], [1,2,3])
        self.interpolateQuaternion(td1, attID1, 4)
        other.interpolateQuaternion(td2, attID2, 4)
        
        rotation = Quaternion.q_mult(Quaternion.q_mult(Quaternion.inverted(td2.D[0,4:8]),q_CB),td1.D[0,4:8]);
        translation = td1.D[0,1:4]+Quaternion.rotate(Quaternion.inverse(td1.D[0,4:8]),B_r_BC)-Quaternion.rotate(Quaternion.inverse(rotation),td2.D[0,1:4])
        
        return translation, rotation
    
    @classmethod
    def basicTests(self):
        td1 = TimedData(4)
        print('Array with times 1-5:')
        td1.initEmptyFromTimes(np.array([1,2,3,4,5]))
        print(td1.D())
        print('Set second row to 1,2,3,4')
        td1.setRow(np.array([1,2,3,4]),1)
        print(td1.D())
        print('Try setting second row to 1,2,3,4,5')
        td1.setRow(np.array([1,2,3,4,5]),1)
        print('Try setting 6th row to 1,2,3,4')
        td1.setRow(np.array([1,2,3,4]),5)
        print('Set second column to 10,11,12,13,14')
        td1.setCol(np.array([10,11,12,13,14]),1)
        print(td1.D())
        print('Try setting second column to 1,2,3,4')
        td1.setCol(np.array([1,2,3,4]),4)
        print('Try setting 5th column to 1,2,3,4,5')
        td1.setCol(np.array([1,2,3,4,5]),4)
        print('Set block to [[1,2],[3,4]] at 2,2')
        td1.setBlock(np.array([[1,2],[3,4]]),2,2)
        print(td1.D())
        print('Get first row')
        print(td1.row(0))
        print('Get second col')
        print(td1.col(1))
        print('Last entry is at: ')
        print(td1.last)
        print('Length is:')
        print(td1.end())
        print('Append Data (duplicates array):')
        td1.append()
        print('Total Array:')
        print(td1.d)
        print('Reduced (meaningful) part of the array')
        print(td1.D())
        print('Last entry is at: ')
        print(td1.last)
        print('Length is:')
        print(td1.end())

    @classmethod
    def advancedTests(self):
        td1 = TimedData(15)
        td2 = TimedData(15)
        td1.initEmptyFromTimes([1,2,3,4])
        td2.initEmptyFromTimes([0,1.5,2.5,5])
        # Set Position
        td1.setBlock([[1,2,3],[2,3,2],[4,2,1],[3,4,3]], 0, 1)
        # Set Quaternion
        q11 = Quaternion.q_exp(np.array([0.1,0.2,0.3]))
        q12 = Quaternion.q_exp(np.array([0.1,0.2,0.32]))
        q13 = Quaternion.q_exp(np.array([0.1,0.2,0.33]))
        q14 = Quaternion.q_exp(np.array([0.1,0.2,0.36]))
        q21 = Quaternion.q_exp(np.array([0.5,0.2,0.3]))
        q22 = Quaternion.q_exp(np.array([0.4,0.2,0.32]))
        q23 = Quaternion.q_exp(np.array([0.3,0.2,0.33]))
        q24 = Quaternion.q_exp(np.array([0.1,0.2,0.36]))

        td1.setBlock(np.array([q11,q12,q13,q14]), 0, 4)
        td2.setBlock(np.array([q21,q22,q23,q24]), 0, 4)

        print('td 1:')
        print(td1.D())
        print('td 2: to interpolate')
        print(td2.D())
        td1.interpolateColumns(td2, [1,3],[2,1])
        print('td 2: interpolated')
        print(td2.D())
        print('td 2: to interpolate')
        print(td2.D())
        td1.interpolateQuaternion(td2, 4,4)
        print('td 2: interpolated')
        print(td2.D())
        td1.computeVectorNDerivative([1,2,3], [8,9,10])
        print('td 1: VEL')
        print(td1.D())
        td1.computeRotationalRateFromAttitude(4, 11)
        td2.computeRotationalRateFromAttitude(4, 11)
        print('td 1: ROR')
        print(td1.D())
        print('Time Offset')
        print(td1.getTimeOffset(td2, 11,11))
        