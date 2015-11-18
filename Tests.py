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
        np.testing.assert_almost_equal(Quaternion.q_log(q1), v1, decimal=7)
        np.testing.assert_almost_equal(Quaternion.q_rotate(q1,v2), np.resize(Quaternion.q_toRotMat(q1),(3,3)).dot(v2), decimal=7)
        for i in np.arange(0,3):
            dv1 = np.array([0.0, 0.0, 0.0])
            dv1[i] = 1.0
            epsilon = 1e-6
            ypr1 = Quaternion.q_toYpr(q1)
            ypr1_dist = Quaternion.q_toYpr(Quaternion.q_boxPlus(q1,dv1*epsilon))
            dypr1_1 = (ypr1_dist-ypr1)/epsilon
            J = np.resize(Quaternion.q_toYprJac(q1),(3,3))
            dypr1_2 = J.dot(dv1)
            np.testing.assert_almost_equal(dypr1_1,dypr1_2, decimal=5)
    def test_TimedDataClass(self):
        print('TODO')

if __name__ == '__main__':
    unittest.main()