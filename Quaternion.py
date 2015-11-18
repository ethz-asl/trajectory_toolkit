import numpy as np
import Utils

def q_mult(q1, q2): #TESTED
    q = np.zeros_like(q1)
    q.T[0,] = (q1.T[0,]*q2.T[0,] - q1.T[1,]*q2.T[1,] - q1.T[2,]*q2.T[2,] - q1.T[3,]*q2.T[3,])
    q.T[1,] = (q1.T[0,]*q2.T[1,] + q1.T[1,]*q2.T[0,] - q1.T[2,]*q2.T[3,] + q1.T[3,]*q2.T[2,])
    q.T[2,] = (q1.T[0,]*q2.T[2,] + q1.T[1,]*q2.T[3,] + q1.T[2,]*q2.T[0,] - q1.T[3,]*q2.T[1,])
    q.T[3,] = (q1.T[0,]*q2.T[3,] - q1.T[1,]*q2.T[2,] + q1.T[2,]*q2.T[1,] + q1.T[3,]*q2.T[0,])
    return q

def q_Lmat(q):
    L = np.zeros([4,4])
    L[0,0] = q.T[0,]
    L[0,1] = -q.T[1,]
    L[0,2] = -q.T[2,]
    L[0,3] = -q.T[3,]
    L[1,0] = q.T[1,]
    L[1,1] = q.T[0,]
    L[1,2] = q.T[3,]
    L[1,3] = -q.T[2,]
    L[2,0] = q.T[2,]
    L[2,1] = -q.T[3,]
    L[2,2] = q.T[0,]
    L[2,3] = q.T[1,]
    L[3,0] = q.T[3,]
    L[3,1] = q.T[2,]
    L[3,2] = -q.T[1,]
    L[3,3] = q.T[0,]
    return L

def q_Rmat(q):
    R = np.zeros([4,4])
    R[0,0] = q.T[0,]
    R[0,1] = -q.T[1,]
    R[0,2] = -q.T[2,]
    R[0,3] = -q.T[3,]
    R[1,0] = q.T[1,]
    R[1,1] = q.T[0,]
    R[1,2] = -q.T[3,]
    R[1,3] = q.T[2,]
    R[2,0] = q.T[2,]
    R[2,1] = q.T[3,]
    R[2,2] = q.T[0,]
    R[2,3] = -q.T[1,]
    R[3,0] = q.T[3,]
    R[3,1] = -q.T[2,]
    R[3,2] = q.T[1,]
    R[3,3] = q.T[0,]
    return R

def q_invert(q): #TESTED
    q.T[1:4,] *= -1

def q_inverse(q): #TESTED
    qInv = np.copy(q)
    q_invert(qInv)
    return qInv

def q_norm(q): #TESTED
    return Utils.norm(q)

def q_normalize(q): 
    print('Not implemented!')
    
def q_normalized(q): #TESTED
    return (q.T[0:4,] / q_norm(q)).T

def q_rotate(q1, v1): #TESTED
    q2 = np.zeros_like(q1)
    q2.T[1:4,] = v1.T;
    return (q_mult(q_mult(q1, q2), q_inverse(q1)).T[1:,]).T

def q_log(q): #TESTED
    root = np.sqrt(np.maximum(1.0-q.T[0,]*q.T[0,],1e-6))
    theta = 2.0*np.arccos(np.maximum(np.minimum(q.T[0,],1.0),-1.0))
    return (theta/root*q.T[1:4,]).T;

def q_exp(v): #TESTED
    # TODO: find better solution?
    if v.ndim == 1:
        q = np.zeros([4])
    else:
        q = np.zeros([np.shape(v)[0],4])
    theta = Utils.norm(v);
    q.T[1:4,] = (np.sin(theta*0.5) / np.maximum(theta,1e-6)*v.T)
    q.T[0,] = np.cos(theta*0.5).T
    return q

def q_boxPlus(q,v): #TESTED
    return q_mult(q_exp(v),q)
    
def q_boxMinus(q1,q2): #TESTED
    return q_log(q_mult(q1,q_inverse(q2)))
    
def q_slerp(q1,q2,t): #TESTED
    return q_boxPlus(q1, (t*q_boxMinus(q2, q1)))

def q_mean(q):
    if q.ndim == 1:
        return q;
    else:
        n = np.shape(q)[0]
        mean_v = (np.sum(q_boxMinus(q[1:n,],np.kron(np.ones([n-1,1]),q[0,])), axis=0)/n);
        return q_boxPlus(q[0,], mean_v)

def q_toYpr(q):
    if q.ndim == 1:
        ypr = np.zeros([3])
    else:
        ypr = np.zeros([np.shape(q)[0],3])
#     ypr.T[2,] = -np.arctan2(2*(q.T[1,]*q.T[2,]-q.T[0,]*q.T[3,]),1-2*(q.T[2,]*q.T[2,]+q.T[3,]*q.T[3,]))
#     ypr.T[1,] = np.arcsin(2*(q.T[1,]*q.T[3,]+q.T[0,]*q.T[2,]))
#     ypr.T[0,] = -np.arctan2(2*(q.T[2,]*q.T[3,]-q.T[0,]*q.T[1,]),1-2*(q.T[1,]*q.T[1,]+q.T[2,]*q.T[2,]))
    ypr.T[0,] = -np.arctan2(2*(q.T[1,]*q.T[2,]-q.T[0,]*q.T[3,]),1-2*(q.T[2,]*q.T[2,]+q.T[3,]*q.T[3,]))
    ypr.T[1,] = np.arcsin(2*(q.T[1,]*q.T[3,]+q.T[0,]*q.T[2,]))
    ypr.T[2,] = -np.arctan2(2*(q.T[2,]*q.T[3,]-q.T[0,]*q.T[1,]),1-2*(q.T[1,]*q.T[1,]+q.T[2,]*q.T[2,]))
    return ypr

def q_toYprJac(q):
    if q.ndim == 1:
        J = np.zeros([9])
    else:
        J = np.zeros([np.shape(q)[0],9])
    ypr = q_toYpr(q)
    t2 = np.cos(ypr.T[1,])
    t3 = 1.0/t2
    t4 = np.cos(ypr.T[2,])
    t5 = np.sin(ypr.T[2,])
    t6 = np.sin(ypr.T[1,])
#     J.T[1,] = -t3*t5 # TODO: this fits unit test, the next seems better with covariance
#     J.T[0,] = t3*t4
#     J.T[4,] = t4
#     J.T[3,] = t5
#     J.T[8,] = 1.0
#     J.T[7,] = t3*t5*t6
#     J.T[6,] = -t3*t4*t6
    J.T[1,] = t3*t5
    J.T[2,] = t3*t4
    J.T[4,] = t4
    J.T[5,] = -t5
    J.T[6,] = 1.0
    J.T[7,] = t3*t5*t6
    J.T[8,] = t3*t4*t6
    return J

def q_toRotMat(q):
    if q.ndim == 1:
        R = np.zeros([9])
    else:
        R = np.zeros([np.shape(q)[0],9])
    
    R.T[0,] = 1.0 - 2*q.T[2,]*q.T[2,] - 2*q.T[3,]*q.T[3,]
    R.T[1,] = 2*(q.T[1,]*q.T[2,]+q.T[3,]*q.T[0,])
    R.T[2,] = 2*(q.T[1,]*q.T[3,]-q.T[2,]*q.T[0,])
    R.T[3,] = 2*(q.T[1,]*q.T[2,]-q.T[3,]*q.T[0,])
    R.T[4,] = 1.0 - 2*q.T[1,]*q.T[1,] - 2*q.T[3,]*q.T[3,]
    R.T[5,] = 2*(q.T[2,]*q.T[3,]+q.T[1,]*q.T[0,])
    R.T[6,] = 2*(q.T[1,]*q.T[3,]+q.T[2,]*q.T[0,])
    R.T[7,] = 2*(q.T[2,]*q.T[3,]-q.T[1,]*q.T[0,])
    R.T[8,] = 1.0 - 2*q.T[1,]*q.T[1,] - 2*q.T[2,]*q.T[2,]
    
    return R