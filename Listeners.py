import rospy
from geometry_msgs.msg import TransformStamped
import numpy as np
from TimedData import TimedData

def addTransformStamped(td,msg,translationID,rotationID):
    td.t.append(msg.header.stamp.to_sec())
    d = np.zeros(td.N)
    d[translationID:translationID+3] = [msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z];
    d[rotationID:rotationID+4] = [msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w];
    td.d.append(d)

class TransformStampedListener:
    td = TimedData(0)
    translationID = 0
    rotationID = 3
    def __init__(self, td, topic, translationID = 0, rotationID = 3):
        self.td = td
        self.translationID = translationID
        self.rotationID = rotationID
        rospy.Subscriber(topic, TransformStamped, self.callback)
    def callback(self,msg):
        addTransformStamped(self.td, msg, self.translationID, self.rotationID)