import numpy as np
def norm(v): #TESTED
    return np.sum(v**2,axis=-1)**(1./2)