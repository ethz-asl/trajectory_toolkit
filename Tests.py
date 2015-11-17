import unittest
from TimedData import TimedData
import numpy as np
import Quaternion


class TestTimedData(unittest.TestCase):

    def test_QuaternionClass(self):        
        v1 = np.array([0.2, 0.2, 0.4])
        v2 = np.array([1, 0, 0])         
        q1 = Quaternion.q_exp(v1)
        q2 = Quaternion.q_exp(v2)
        v=np.array([1, 2, 3])
        np.testing.assert_almost_equal(Quaternion.q_rotate(Quaternion.q_mult(q1,q2),v), Quaternion.q_rotate(q1,Quaternion.q_rotate(q2,v)), decimal=7)
        np.testing.assert_almost_equal(Quaternion.q_boxPlus(q1,Quaternion.q_boxMinus(q2,q1)), q2, decimal=7)
        np.testing.assert_almost_equal(Quaternion.q_log(Quaternion.q_exp(v1)), v1, decimal=7)
        
    def test_TimedDataClass(self):

if __name__ == '__main__':
    unittest.main()