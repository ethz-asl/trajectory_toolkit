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
    
    def setCol(self, data, columnID): #TESTED
        if(np.shape(data)[0] == self.end()):
            if(columnID < self.Nc):    
                self.d[:self.end(),columnID] = data;
            else:
                print(colored('WARNING: Did not set column! No column with that ID!','yellow'))
        else:
            print(colored('WARNING: Did not set column! Wrong data size!','yellow'));
    
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
    
    def col(self, columnID): #TESTED
        # Check columnID validity
        if (columnID > (np.shape(self.d)[1]-1) ):
            print(colored('WARNING: You requested an invalid columnID = '+str(columnID) + '. Returning Zeros!','yellow'));
            return np.zeros([self.end()]); 
        # Return data
        return self.d[0:self.end(), columnID];

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
    def computeDerivativeOfColumn(self, dataID, derivativeID): #TESTED
        dp = np.diff(self.col(dataID))
        dt = np.diff(self.getTime())
        self.d[1:self.end(),derivativeID] = np.divide(dp,dt);

    def computeVector3Derivative(self, inputID, outputID): #TESTED
        self.computeDerivativeOfColumn(inputID,   outputID);
        self.computeDerivativeOfColumn(inputID+1, outputID+1);
        self.computeDerivativeOfColumn(inputID+2, outputID+2);
        
    def computeRotationalRateFromAttitude(self, attitudeID, rotationalrateID):
        dv = Quaternion.q_boxMinus(self.d[1:self.end(),attitudeID:attitudeID+4],self.d[0:self.last,attitudeID:attitudeID+4])
        dt = np.diff(self.getTime())
        self.d[1:self.end(),rotationalrateID:rotationalrateID+3] = np.divide(dv,dt);
        
    def interpolateColumns(self, tdOut, tdOutColIDs, colIDs=None): #TESTED
        # Allow interpolating in to other columns / default is into same column
        if colIDs is None:
            colIDs = tdOutColIDs
        # ColIDs must match in size to be interpolated
        if(len(tdOutColIDs)==len(colIDs)):
            for i in xrange(0,len(tdOutColIDs)):
                self.interpolateColumn(tdOut, colIDs[i], tdOutColIDs[i])
        else:
            print(colored('WARNING: Interpolation failed! ColIDs did not match in size.','yellow'))

    def interpolateColumn(self, tdOut, tdOutColID, colID=None): #TESTED
        # NOTE: Values outside of the timerange of self are set to the first rsp. last value (no extrapolation)
        # Allow interpolating in to other columns / default is into same column
        if colID is None:
            colID = tdOutColID
        # When times are increasing interpolate using built in numpy interpolation
        if(np.all(np.diff(tdOut.getTime()) > 0)):
            tdOut.setCol(np.interp(tdOut.getTime(), self.getTime(), self.col(colID)),tdOutColID)
        else:
            print(colored('WARNING: Interpolation failed! Time values must be increasing.','yellow'))
    
    def interpolateQuaternion(self, tdOut, tdOutColID, colID):
        # All quaternions before self start are set to the first entry
        counter = 0;
        while tdOut.col(tdOut.timeID)[counter] <= self.col(self.timeID[0] ):
            tdOut.d[counter,tdOutColID:tdOutColID+4] = self.d[0,colID:colID+4]
            counter += 1
        # Interpolation 
        while (counter < tdOut.end()):
            Quaternion.q_slerp()
        
    def getTimeOffset(self,tdIn, colID, tdInColID=None):
        paddingWithRMS = False
        weightingWithOverlapLength = False
        if tdInColID is None:
            tdInColID = colID
        # Make timing calculation
        dtIn = tdIn.getLastTime()-tdIn.getFirstTime()
        dt = self.getLastTime()-self.getFirstTime()
        timeIncrement = min(dt/(self.length()-1), dtIn/(tdIn.length()-1))
        N = math.ceil(dt/timeIncrement)+1
        NIn = math.ceil(dtIn/timeIncrement)+1
        # Interpolate of trajectories
        td1 = TimedData(2);
        td2 = TimedData(2);
        if paddingWithRMS:
            td1.initEmptyFromTimes(self.getFirstTime()+np.arange(-(NIn-1),N+(NIn-1))*timeIncrement)
        else:
            td1.initEmptyFromTimes(self.getFirstTime()+np.arange(N)*timeIncrement)
        td2.initEmptyFromTimes(tdIn.getFirstTime()+np.arange(NIn)*timeIncrement)
        self.interpolateColumn(td1, 1, colID)
        tdIn.interpolateColumn(td2, 1, tdInColID)
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
        return timeIncrement*n + self.getFirstTime() - tdIn.getFirstTime()
    
    def applyTimeOffset(self,to):
        self.setCol(self.col(0) + to,0)
        
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
        td1.computeVector3Derivative(1, 8)
        print('td 1: VEL')
        print(td1.D())
        td1.computeRotationalRateFromAttitude(4, 11)
        td2.computeRotationalRateFromAttitude(4, 11)
        print('td 1: ROR')
        print(td1.D())
        print('Time Offset')
        print(td1.getTimeOffset(td2, 11,11))
        