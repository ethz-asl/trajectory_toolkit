import numpy as np
import Quaternion
import time

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
    
    # These getter are used to access the "meaningful" data
    def d(self):
        return self.d[0:self.last, :];
    
    def col(self, columnID):
        # Check columnID validity
        if (columnID > np.shape(self.d)[1] ):
            print('You requested an invalid columnID = '+str(columnID) + '. Returning Zeros!');
            return np.zeros([self.last+1,1]); 
        # Return data
        return self.d[0:self.last, columnID];

    def row(self, rowID):
        # Check rowID validity
        if(rowID > self.last):
            print('You requested an invalid rowID = '+str(rowID) + '. Returning Zeros!');
            return np.zeros([1,self.Nc]); 
        # Return data
        return self.d[rowID,:];     
   
    # Append an entry, resize array
    def append(self):
        self.last += 1;
        if np.shape(self.d)[0] == self.last:
            self.d = np.resize(self.d, (2*np.shape(self.d)[0], self.Nc));   
    
    def length(self):
        return (self.last + 1);
    # Math functions
    def computeDerivativeOfColumn(self, dataID, derivativeID):
        # NOTE: The array is longer than its data!
        np.divide(np.diff(self.col(dataID)), np.diff(self.col(self.timeID)), self.d[1:self.last, derivativeID]);

    def computeVeloctiyFromPosition(self, positonID, velocityID):
        self.computeDerivativeOfColumn(positonID, velocityID);
        self.computeDerivativeOfColumn(positonID+1, velocityID+1);
        self.computeDerivativeOfColumn(positonID+2, velocityID+2);
    
    def test(self):
#         n=1000000
#         q = np.ones([n,4])
#         for i in range(0,n):
#             q1[i,:] = Quaternion.q_inverse(q[i,:])
        q2 = np.array([[1, 0, 1, 0],[1, 0, 1, 0]])
        q1 = np.array([Quaternion.q_normalized(np.array([1, 0, 0, 1])),[1, 0, 1, 0]])
        v = np.array([[1, 0, 0],[4,5,6]])
        print(Quaternion.q_rotate(q1,v))
        #Quaternion.q_normalize(q)
        #print(q)


        