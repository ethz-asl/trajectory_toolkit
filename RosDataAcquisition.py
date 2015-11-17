import rospy
from geometry_msgs.msg import TransformStamped
import numpy as np
import rosbag as rb
from TimedData import TimedData

def addTransformStamped(td, msg, posID, attID):
    td.append();
    td.d[td.last, td.timeID] = msg.header.stamp.to_sec();
    td.d[td.last, posID:posID+3] = np.array([msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z]);
    td.d[td.last, attID:attID+4] = np.array([msg.transform.rotation.w,msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z]);

def addOdometry(td, msg, posID = None, attID = None, velID = None, rorID = None, posCovID = None, attCovID = None, velCovID = None, rorCovID = None):
    td.append();
    td.d[td.last, td.timeID] = msg.header.stamp.to_sec();
    if posID != None:
        td.d[td.last, posID:posID+3] = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]);
    if attID != None:
        td.d[td.last, attID:attID+4] = np.array([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z]);
    if velID != None:
        td.d[td.last, velID:velID+3] = np.array([msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z]);
    if rorID != None:
        td.d[td.last, rorID:rorID+3] = np.array([msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z]);
    if posCovID != None:
        td.d[td.last, posCovID:posCovID+9] = np.array([msg.pose.covariance[0],msg.pose.covariance[1],msg.pose.covariance[2],
                                                       msg.pose.covariance[6],msg.pose.covariance[7],msg.pose.covariance[8],
                                                       msg.pose.covariance[12],msg.pose.covariance[13],msg.pose.covariance[14]]);
    if attCovID != None:
        td.d[td.last, attCovID:attCovID+9] = np.array([msg.pose.covariance[21],msg.pose.covariance[22],msg.pose.covariance[23],
                                                       msg.pose.covariance[27],msg.pose.covariance[28],msg.pose.covariance[29],
                                                       msg.pose.covariance[33],msg.pose.covariance[34],msg.pose.covariance[35]]);
    if velCovID != None:
        td.d[td.last, velCovID:velCovID+9] = np.array([msg.twist.covariance[0],msg.twist.covariance[1],msg.twist.covariance[2],
                                                       msg.twist.covariance[6],msg.twist.covariance[7],msg.twist.covariance[8],
                                                       msg.twist.covariance[12],msg.twist.covariance[13],msg.twist.covariance[14]]);
    if rorCovID != None:
        td.d[td.last, rorCovID:rorCovID+9] = np.array([msg.twist.covariance[21],msg.twist.covariance[22],msg.twist.covariance[23],
                                                       msg.twist.covariance[27],msg.twist.covariance[28],msg.twist.covariance[29],
                                                       msg.twist.covariance[33],msg.twist.covariance[34],msg.twist.covariance[35]]);

# Listener on a Trans
class TransformStampedListener:
    td = TimedData(0)
    posID = 1
    attID = 4
    
    def __init__(self, td, topic, posID = 1, attID = 4):
        self.td = td
        self.posID = posID
        self.attID = attID
        rospy.Subscriber(topic, TransformStamped, self.callback)
    def callback(self,msg):
        addTransformStamped(self.td, msg, self.posID, self.attID)
    
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
            
    def loadTransformStamped(self, td, posID, attID):
        # If timedata is empty fill it up with the transforms
        if( td.last == (-1) ):
            for topic, msg, t in self.bag.read_messages(topics=[self.topic]):
                addTransformStamped(td, msg, posID, attID)
        else:
            print('Implement functionality when timedata is not empty');
            
def rosBagCountTopic(bag, topic):
    counter = 0
    for top, msg, t in bag.read_messages(topics=[topic]):
        counter = counter + 1
    return counter

def rosBagLoadTransformStamped(filename, topic, td, posID, attID):
    bag = rb.Bag(filename)
    count = rosBagCountTopic(bag,topic)
    print("loading " + filename + ", found " + str(count) +" "+ topic +" entries")
    if( td.last == (-1) ):
        for top, msg, t in bag.read_messages(topics=[topic]):
            addTransformStamped(td, msg, posID, attID)
    else:
        print('Implement functionality when timedata is not empty');

def rosBagLoadOdometry(filename, topic, td, posID = None, attID = None, velID = None, rorID = None, posCovID = None, attCovID = None, velCovID = None, rorCovID = None):
    bag = rb.Bag(filename)
    count = rosBagCountTopic(bag,topic)
    print("loading " + filename + ", found " + str(count) +" "+ topic +" entries")
    if( td.last == (-1) ):
        for top, msg, t in bag.read_messages(topics=[topic]):
            addOdometry(td, msg, posID, attID, velID, rorID, posCovID, attCovID, velCovID, rorCovID)
    else:
        print('Implement functionality when timedata is not empty');
    
    
    
    
    
    
    