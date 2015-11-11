import rospy
from geometry_msgs.msg import TransformStamped
import numpy as np
import rosbag as rb
from TimedData import TimedData

# Common Functionality for all acquisition methods
def addTransformStamped(td, msg, translationID, rotationID):
    td.append();
    td.d[td.last, td.timeID] = msg.header.stamp.to_sec();
    td.d[td.last, translationID:translationID+3] = np.array([msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z]);
    td.d[td.last, rotationID:rotationID+4] = np.array([msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w]);

# Listener on a Trans
class TransformStampedListener:
    td = TimedData(0)
    translationID = 1
    rotationID = 4
    
    def __init__(self, td, topic, translationID = 1, rotationID = 4):
        self.td = td
        self.translationID = translationID
        self.rotationID = rotationID
        rospy.Subscriber(topic, TransformStamped, self.callback)
    def callback(self,msg):
        addTransformStamped(self.td, msg, self.translationID, self.rotationID)
    
class RosbagStampedTopicLoader:
    # TODO: Provide Functionality that allows changing of bagfile and topic etc.
    bag = []
    stamp = []
    topic = []
    
    def __init__(self, filename, topic):
        # Get rosbag
        self.bag = rb.Bag(filename)
        self.topic = topic
        # Count number of entries
        counter = 0
        for topic, msg, t in self.bag.read_messages(topics=[self.topic]):
            counter = counter + 1
        print("loading " + filename + ", found " + str(counter) +" "+ self.topic +" entries")
        # Allocate time vector
        self.stamp = np.empty([counter,1])
        # Fill time array
        counter = 0
        for topic, msg, t in self.bag.read_messages(topics=[self.topic]):
            self.stamp[counter, 0] = msg.header.stamp.to_sec();
            counter = counter + 1
            
    def loadTransformStamped(self, td, translationID, rotationID):
        # If timedata is empty fill it up with the transforms
        if( td.last == (-1) ):
            for topic, msg, t in self.bag.read_messages(topics=[self.topic]):
                addTransformStamped(td, msg, translationID, rotationID)
        else:
            print('Implement functionality when timedata is not empty');