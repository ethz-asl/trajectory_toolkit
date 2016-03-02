import unittest
from TimedData import TimedData
import numpy as np
import Quaternion
from cmath import sqrt


class TestTimedData(unittest.TestCase):

    def test_QuaternionClass(self):        
        v1 = np.array([0.1, -0.2, 0.4])
        v2 = np.array([1, 0, 0])         
        q1 = Quaternion.q_exp(v1)
        q2 = Quaternion.q_exp(v2)
        v=np.array([1, 2, 3])
        ########################### Testing Mult and rotate and RotMat ###########################
        np.testing.assert_almost_equal(Quaternion.q_rotate(Quaternion.q_mult(q1,q2),v), Quaternion.q_rotate(q1,Quaternion.q_rotate(q2,v)), decimal=7)
        np.testing.assert_almost_equal(Quaternion.q_rotate(q1,v2), np.resize(Quaternion.q_toRotMat(q1),(3,3)).dot(v2), decimal=7)
        ########################### Testing RotMat ###########################
        np.testing.assert_almost_equal(Quaternion.q_rotMatToQuat(Quaternion.q_toRotMat(q1)), q1, decimal=7)
        ########################### Testing Boxplus, Boxminus, Log and Exp ###########################
        np.testing.assert_almost_equal(Quaternion.q_boxPlus(q1,Quaternion.q_boxMinus(q2,q1)), q2, decimal=7)
        np.testing.assert_almost_equal(Quaternion.q_log(q1), v1, decimal=7)
        ########################### Testing Exp and RotMat ###########################
        np.testing.assert_almost_equal(Quaternion.q_rotVecToRotMat(v1), Quaternion.q_toRotMat(Quaternion.q_exp(v1)), decimal=7)
        ########################### Testing mult/exp/rotate ###########################
        np.testing.assert_almost_equal(Quaternion.q_mult(q1, Quaternion.q_mult(Quaternion.q_exp(v2),Quaternion.q_inverse(q1))),Quaternion.q_exp(Quaternion.q_rotate(q1, v2)), decimal=7)
        ########################### Testing Lmat and Rmat ###########################
        np.testing.assert_almost_equal(Quaternion.q_mult(q1,q2), Quaternion.q_Lmat(q1).dot(q2), decimal=7)
        np.testing.assert_almost_equal(Quaternion.q_mult(q1,q2), Quaternion.q_Rmat(q2).dot(q1), decimal=7)
        ########################### Testing ypr and quat ###########################
        roll = 0.2
        pitch = -0.5
        yaw = 2.5
        q_test = Quaternion.q_mult(np.array([np.cos(0.5*pitch), 0, np.sin(0.5*pitch), 0]),np.array([np.cos(0.5*yaw), 0, 0, np.sin(0.5*yaw)]))
        q_test = Quaternion.q_mult(np.array([np.cos(0.5*roll), np.sin(0.5*roll), 0, 0]),q_test)
        np.testing.assert_almost_equal(Quaternion.q_toYpr(q_test), np.array([roll, pitch, yaw]), decimal=7)
        ########################### Testing Jacobian of Ypr ###########################
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
        ########################### Testing Jacobian of Exp ###########################
        for i in np.arange(0,3):
            dv1 = np.array([0.0, 0.0, 0.0])
            dv1[i] = 1.0
            epsilon = 1e-6
            q1 = Quaternion.q_exp(v1)
            q1_dist = Quaternion.q_exp(v1+dv1*epsilon)
            dq1_1 = Quaternion.q_boxMinus(q1_dist, q1)/epsilon
            J = np.resize(Quaternion.q_rotVecToGamma(v1),(3,3))
            dq1_2 = J.dot(dv1)
            np.testing.assert_almost_equal(dq1_1,dq1_2, decimal=5)
    def test_TimedDataClass(self):
        print('TODO')

if __name__ == '__main__':
    unittest.main()