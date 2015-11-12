import numpy as np

def q_mult(q1, q2): #TESTED
    q = np.zeros_like(q1)
    q.T[0,] = (q1.T[0,]*q2.T[0,] - q1.T[1,]*q2.T[1,] - q1.T[2,]*q2.T[2,] - q1.T[3,]*q2.T[3,])
    q.T[1,] = (q1.T[0,]*q2.T[1,] + q1.T[1,]*q2.T[0,] - q1.T[2,]*q2.T[3,] + q1.T[3,]*q2.T[2,])
    q.T[2,] = (q1.T[0,]*q2.T[2,] + q1.T[1,]*q2.T[3,] + q1.T[2,]*q2.T[0,] - q1.T[3,]*q2.T[1,])
    q.T[3,] = (q1.T[0,]*q2.T[3,] - q1.T[1,]*q2.T[2,] + q1.T[2,]*q2.T[1,] + q1.T[3,]*q2.T[0,])
    return q

def q_invert(q): #TESTED
    q.T[1:4,] *= -1

def q_inverse(q): #TESTED
    qInv = np.copy(q)
    q_invert(qInv)
    return qInv

def q_norm(q): #TESTED
    return np.sum(np.abs(q)**2,axis=-1)**(1./2)

def q_normalize(q):
    print('Not implemented yet!')
    #q.T[0:4,] /= q_norm(q)
    
def q_normalized(q): #TESTED
    return q.T[0:4,] / q_norm(q)

def q_rotate(q1, v1): #TESTED
    q2 = np.zeros_like(q1)
    q2.T[1:4,] = v1.T;
    return (q_mult(q_mult(q1, q2), q_inverse(q1)).T[1:,]).T

def q_boxPlus(q1,q2):
    print('Not implemented yet!')
    
def q_boxMinus(q1,q2):
    print('Not implemented yet!')
    
def q_slerp(q1,q2,t):
    return q_boxPlus(q1, (t*q_boxMinus(q2, q1)))