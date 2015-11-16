import numpy as np
def norm(v): #TESTED
    return np.sum(v**2,axis=-1)**(1./2)

def skew(v):
    skew = np.zeros([3,3])
    skew[0,1] = -v[2]
    skew[0,2] =  v[1]
    skew[1,0] =  v[2]
    skew[1,2] = -v[0]
    skew[2,0] = -v[1]
    skew[2,1] =  v[0]
    return skew
    