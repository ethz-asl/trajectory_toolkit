import numpy as np
import Quaternion
import time
from __builtin__ import classmethod
from termcolor import colored

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
    
    def initEmptyFromTimes(self, times, Nc): #TESTED
        self.last = np.shape(times)[0]-1
        self.Nc = Nc
        self.d = np.zeros([np.shape(times)[0], Nc])
        self.d[:self.end(),self.timeID] = times;
    
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
   
    # Append an entry, resize array
    def append(self): #TESTED
        if np.shape(self.d)[0] == self.end():
            self.d = np.resize(self.d, (2*np.shape(self.d)[0], self.Nc));   
        self.last += 1;

    def end(self): #TESTED
        return (self.last + 1);
    
    # Math functions
    def computeDerivativeOfColumn(self, dataID, derivativeID):
        self.d[1:self.end(),derivativeID] = np.divide(np.diff(self.col(dataID)), np.diff(self.col(self.timeID)));

    def computeVeloctiyFromPosition(self, positonID, velocityID):
        self.computeDerivativeOfColumn(positonID, velocityID);
        self.computeDerivativeOfColumn(positonID+1, velocityID+1);
        self.computeDerivativeOfColumn(positonID+2, velocityID+2);
    
    def interpolateColumns(self, tdOut, colIDs): #TESTED
        for colID in colIDs:
            self.interpolateColumn(tdOut, colID)
    
    def interpolateColumn(self, tdOut, colID): #TESTED
        # NOTE: Values outside of the timerange of self are set to the first rsp. last value (no extrapolation)
        if(np.all(np.diff(tdOut.col(tdOut.timeID)) > 0)):
            tdOut.setCol(np.interp(tdOut.col(tdOut.timeID), self.col(self.timeID), self.col(colID)),colID)
        else:
            print(colored('WARNING: Interpolation of column '+str(colID)+' failed! Time values must be increasing.','yellow'))

    def getTimeOffset(self,tdIn):
        return
    
    
    def basicTests(self):
        print('Array with times 1-5:')
        self.initEmptyFromTimes(np.array([1,2,3,4,5]), 4)
        print(self.D())
        print('Set second row to 1,2,3,4')
        self.setRow(np.array([1,2,3,4]),1)
        print(self.D())
        print('Try setting second row to 1,2,3,4,5')
        self.setRow(np.array([1,2,3,4,5]),1)
        print('Try setting 6th row to 1,2,3,4')
        self.setRow(np.array([1,2,3,4]),5)
        print('Set second column to 10,11,12,13,14')
        self.setCol(np.array([10,11,12,13,14]),1)
        print(self.D())
        print('Try setting second column to 1,2,3,4')
        self.setCol(np.array([1,2,3,4]),4)
        print('Try setting 5th column to 1,2,3,4,5')
        self.setCol(np.array([1,2,3,4,5]),4)
        print('Set block to [[1,2],[3,4]] at 2,2')
        self.setBlock(np.array([[1,2],[3,4]]),2,2)
        print(self.D())
        print('Get first row')
        print(self.row(0))
        print('Get second col')
        print(self.col(1))
        print('Last entry is at: ')
        print(self.last)
        print('Length is:')
        print(self.end())
        print('Append Data (duplicates array):')
        self.append()
        print('Total Array:')
        print(self.d)
        print('Reduced (meaningful) part of the array')
        print(self.D())
        print('Last entry is at: ')
        print(self.last)
        print('Length is:')
        print(self.end())
    
    def advancedTests(self):
        td2 = TimedData(4)
        self.initEmptyFromTimes([1,2,3,4], 4)
        td2.initEmptyFromTimes([0,1.5,2.5,5], 4)
        self.setBlock([[1,2,3],[2,3,2],[4,2,1],[3,4,3]], 0, 1)
        print('td 1:')
        print(self.D())
        print('td 2: to interpolate')
        print(td2.D())
        self.interpolateColumns(td2, [1,3])
        print('td 2: interpolated')
        print(td2.D())
        

        