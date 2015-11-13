import numpy as np

def norm(v): #TESTED
    return np.sum(v**2,axis=-1)**(1./2)

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
    return norm(q)

def q_normalize(q): 
    print('Not implemented!')
    
def q_normalized(q): #TESTED
    return (q.T[0:4,] / q_norm(q)).T

def q_rotate(q1, v1): #TESTED
    q2 = np.zeros_like(q1)
    q2.T[1:4,] = v1.T;
    return (q_mult(q_mult(q1, q2), q_inverse(q1)).T[1:,]).T

def q_log(q):
    root = np.sqrt(1.0-q.T[0,]*q.T[0,])
    theta = 2.0*np.arccos(q.T[0,])
    return (theta/root*q.T[1:4,]).T;

def q_exp(v):
    q = np.zeros([np.shape(v)[0],4])
    theta = norm(v);
    q.T[1:4,] = (np.sin(theta*0.5) / theta*v.T)
    q.T[0,] = np.cos(theta*0.5).T
    return q

def q_boxPlus(q,v):
    return q_mult(q_exp(v),q)
    
def q_boxMinus(q1,q2):
    return q_log(q_mult(q1,q_inverse(q2)))
    
def q_slerp(q1,q2,t):
    return q_boxPlus(q1, (t*q_boxMinus(q2, q1)))

def tests():
    v1 = np.array([[0.2, 0.2, 0.4],[0.2,0.4,0.1]])
    v2 = np.array([[1, 0, 0],[0.2,0.4,1]])                
    q1 = q_exp(v1)
    q2 = q_exp(v2)
    v=np.array([[1, 2, 3],[0,0,1]])   
    print('RUN TESTS')
    print('(q1 x q2)(v):')
    print(q_rotate(q_mult(q1,q2),v))
    print('should equal (q1(q2(v)):')
    print(q_rotate(q1,q_rotate(q2,v)))
    print('q1 + (q2-q1)')
    print(q_boxPlus(q1,q_boxMinus(q2,q1)))
    print('should equal q2')
    print(q2)
    print('log(exp(v))')
    print(q_log(q_exp(v1)))
    print('should equal v')
    print(v1)
    print('normalize quaternion [1,2,0,2]')
    
