import rospy
from geometry_msgs.msg import TransformStamped
import numpy as np
import rosbag as rb
from TimedData import TimedData
import struct

def addTransformStamped(td, ind, msg, posID, attID):
    td.d[ind, posID] = np.array([msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z]);
    td.d[ind, attID] = np.array([msg.transform.rotation.w,msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z]);

def addOdometry(td, ind, msg, posID = None, attID = None, velID = None, rorID = None, posCovID = None, attCovID = None, velCovID = None, rorCovID = None):
    if posID != None:
        td.d[ind, posID] = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]);
    if attID != None:
        td.d[ind, attID] = np.array([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z]);
    if velID != None:
        td.d[ind, velID] = np.array([msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z]);
    if rorID != None:
        td.d[ind, rorID] = np.array([msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z]);
    if posCovID != None:
        td.d[ind, posCovID] = np.array([msg.pose.covariance[0],msg.pose.covariance[1],msg.pose.covariance[2],
                                                       msg.pose.covariance[6],msg.pose.covariance[7],msg.pose.covariance[8],
                                                       msg.pose.covariance[12],msg.pose.covariance[13],msg.pose.covariance[14]]);
    if attCovID != None:
        td.d[ind, attCovID] = np.array([msg.pose.covariance[21],msg.pose.covariance[22],msg.pose.covariance[23],
                                                       msg.pose.covariance[27],msg.pose.covariance[28],msg.pose.covariance[29],
                                                       msg.pose.covariance[33],msg.pose.covariance[34],msg.pose.covariance[35]]);
    if velCovID != None:
        td.d[ind, velCovID] = np.array([msg.twist.covariance[0],msg.twist.covariance[1],msg.twist.covariance[2],
                                                       msg.twist.covariance[6],msg.twist.covariance[7],msg.twist.covariance[8],
                                                       msg.twist.covariance[12],msg.twist.covariance[13],msg.twist.covariance[14]]);
    if rorCovID != None:
        td.d[ind, rorCovID] = np.array([msg.twist.covariance[21],msg.twist.covariance[22],msg.twist.covariance[23],
                                                       msg.twist.covariance[27],msg.twist.covariance[28],msg.twist.covariance[29],
                                                       msg.twist.covariance[33],msg.twist.covariance[34],msg.twist.covariance[35]]);

def addRobocentricPointCloud(td, ind, msg, rovio_fea_idxID, rovio_fea_posID):
    nFeatures = len(rovio_fea_posID)
    idField, = [x for x in msg.fields if x.name == 'id']
    xField, = [x for x in msg.fields if x.name == 'x']
    yField, = [x for x in msg.fields if x.name == 'y']
    zField, = [x for x in msg.fields if x.name == 'z']
    step = msg.point_step
    for i in np.arange(nFeatures):
        idValue, = struct.unpack('i', msg.data[i*step+idField.offset:i*step+idField.offset+4])
        xValue, = struct.unpack('f', msg.data[i*step+xField.offset:i*step+xField.offset+4])
        yValue, = struct.unpack('f', msg.data[i*step+yField.offset:i*step+yField.offset+4])
        zValue, = struct.unpack('f', msg.data[i*step+zField.offset:i*step+zField.offset+4])
        td.d[ind, rovio_fea_idxID[i]] = idValue
        td.d[ind, rovio_fea_posID[i]] = np.array([xValue, yValue, zValue])

# Listener on a Trans
class TransformStampedListener:
    td = TimedData(0)
    posID = [1, 2, 3]
    attID = [4, 5, 6, 7]
    
    def __init__(self, td, topic, posID = [1, 2, 3], attID = [4, 5, 6, 7]):
        self.td = td
        self.posID = posID
        self.attID = attID
        rospy.Subscriber(topic, TransformStamped, self.callback)
    def callback(self,msg):
        self.td.append()
        self.td.d[self.td.last, self.td.timeID] = msg.header.stamp.to_sec();
        addTransformStamped(self.td, self.td.last, msg, self.posID, self.attID)
            
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
            td.append()
            td.d[td.last, td.timeID] = msg.header.stamp.to_sec();
            addTransformStamped(td, td.last, msg, posID, attID)
    else:
        print('Implement functionality when timedata is not empty');

def rosBagLoadOdometry(filename, topic, td, posID = None, attID = None, velID = None, rorID = None, posCovID = None, attCovID = None, velCovID = None, rorCovID = None):
    bag = rb.Bag(filename)
    count = rosBagCountTopic(bag,topic)
    print("loading " + filename + ", found " + str(count) +" "+ topic +" entries")
    if( td.last == (-1) ):
        for top, msg, t in bag.read_messages(topics=[topic]):
            td.append()
            td.d[td.last, td.timeID] = msg.header.stamp.to_sec();
            addOdometry(td, td.last, msg, posID, attID, velID, rorID, posCovID, attCovID, velCovID, rorCovID)
    else:
        print('Implement functionality when timedata is not empty');

def rosBagLoadRobocentricPointCloud(filename, topic, td, rovio_fea_idxID, rovio_fea_posID):
    bag = rb.Bag(filename)
    count = rosBagCountTopic(bag,topic)
    print("loading " + filename + ", found " + str(count) +" "+ topic +" entries")
    if( td.last == (-1) ):
        for top, msg, t in bag.read_messages(topics=[topic]):
            td.append()
            td.d[td.last, td.timeID] = msg.header.stamp.to_sec();
            addRobocentricPointCloud(td, td.last, msg, rovio_fea_idxID, rovio_fea_posID)
    else:
        j = 0
        for top, msg, t in bag.read_messages(topics=[topic]):
            while ((j <= td.last) and (td.d[j,td.timeID] < msg.header.stamp.to_sec())):
                j += 1
            if((j <= td.last) and (td.d[j,td.timeID] == msg.header.stamp.to_sec())):
                addRobocentricPointCloud(td, j, msg, rovio_fea_idxID, rovio_fea_posID)
            else:
                print('Could not merge entry into already existing Timed Data');
    
    
    
    
    
    
    