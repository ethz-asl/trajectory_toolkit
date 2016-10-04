import rospy
from geometry_msgs.msg import TransformStamped
import numpy as np
import rosbag as rb
from TimedData import TimedData
import struct

def addTransformStamped(td, ind, msg, pos = None, att = None):
    posID = td.getColIDs(pos)
    if posID != None:
        td.d[ind, posID] = np.array([msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z])
    attID = td.getColIDs(att)
    if attID != None:
        td.d[ind, attID] = np.array([msg.transform.rotation.w,msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z]);
    
def addPoseWithCovariance(td, ind, msg, pos = None, att = None, posCov = None, attCov = None):
    posID = td.getColIDs(pos)
    if posID != None:
        td.d[ind, posID] = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]);
    attID = td.getColIDs(att)
    if attID != None:
        td.d[ind, attID] = np.array([msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z]);
    posCovID = td.getColIDs(posCov)
    if posCovID != None:
        td.d[ind, posCovID] = np.array([msg.pose.covariance[0],msg.pose.covariance[1],msg.pose.covariance[2],
                                                       msg.pose.covariance[6],msg.pose.covariance[7],msg.pose.covariance[8],
                                                       msg.pose.covariance[12],msg.pose.covariance[13],msg.pose.covariance[14]]);
    attCovID = td.getColIDs(attCov)
    if attCovID != None:
        td.d[ind, attCovID] = np.array([msg.pose.covariance[21],msg.pose.covariance[22],msg.pose.covariance[23],
                                                       msg.pose.covariance[27],msg.pose.covariance[28],msg.pose.covariance[29],
                                                       msg.pose.covariance[33],msg.pose.covariance[34],msg.pose.covariance[35]]);
    
def addTwistWithCovariance(td, ind, msg, vel = None, ror = None, velCov = None, rorCov = None):
    velID = td.getColIDs(vel)
    if velID != None:
        td.d[ind, velID] = np.array([msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z]);
    rorID = td.getColIDs(ror)
    if rorID != None:
        td.d[ind, rorID] = np.array([msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z]);
    velCovID = td.getColIDs(velCov)
    if velCovID != None:
        td.d[ind, velCovID] = np.array([msg.twist.covariance[0],msg.twist.covariance[1],msg.twist.covariance[2],
                                                       msg.twist.covariance[6],msg.twist.covariance[7],msg.twist.covariance[8],
                                                       msg.twist.covariance[12],msg.twist.covariance[13],msg.twist.covariance[14]]);
    rorCovID = td.getColIDs(rorCov)
    if rorCovID != None:
        td.d[ind, rorCovID] = np.array([msg.twist.covariance[21],msg.twist.covariance[22],msg.twist.covariance[23],
                                                       msg.twist.covariance[27],msg.twist.covariance[28],msg.twist.covariance[29],
                                                       msg.twist.covariance[33],msg.twist.covariance[34],msg.twist.covariance[35]]);

def addOdometry(td, ind, msg, posID = None, attID = None, velID = None, rorID = None, posCovID = None, attCovID = None, velCovID = None, rorCovID = None):
    addPoseWithCovariance(td,ind,msg,posID,attID,posCovID,attCovID)
    addTwistWithCovariance(td,ind,msg,velID,rorID,velCovID,rorCovID)
    
def addRobocentricPointCloud(td, ind, msg, rovio_fea_idx, rovio_fea_pos, rovio_fea_cov = None, rovio_fea_dis = None, rovio_fea_disCov = None):
    rovio_fea_idxID = td.getColIDs(rovio_fea_idx)
    rovio_fea_posID = td.getColIDs(rovio_fea_pos)
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
        if rovio_fea_idxID != None:
            td.d[ind, rovio_fea_idxID[i]] = idValue
        if rovio_fea_posID != None:
            td.d[ind, rovio_fea_posID[i]] = np.array([xValue, yValue, zValue])
    
    rovio_fea_covID = td.getColIDs(rovio_fea_cov)
    if rovio_fea_covID != None:
        c00Field, = [x for x in msg.fields if x.name == 'c_00']
        c01Field, = [x for x in msg.fields if x.name == 'c_01']
        c02Field, = [x for x in msg.fields if x.name == 'c_02']
        c11Field, = [x for x in msg.fields if x.name == 'c_11']
        c12Field, = [x for x in msg.fields if x.name == 'c_12']
        c22Field, = [x for x in msg.fields if x.name == 'c_22']
        for i in np.arange(nFeatures):
            c00Value, = struct.unpack('f', msg.data[i*step+c00Field.offset:i*step+c00Field.offset+4])
            c01Value, = struct.unpack('f', msg.data[i*step+c01Field.offset:i*step+c01Field.offset+4])
            c02Value, = struct.unpack('f', msg.data[i*step+c02Field.offset:i*step+c02Field.offset+4])
            c11Value, = struct.unpack('f', msg.data[i*step+c11Field.offset:i*step+c11Field.offset+4])
            c12Value, = struct.unpack('f', msg.data[i*step+c12Field.offset:i*step+c12Field.offset+4])
            c22Value, = struct.unpack('f', msg.data[i*step+c22Field.offset:i*step+c22Field.offset+4])
            td.d[ind, rovio_fea_covID[i]] = np.array([c00Value, c01Value, c02Value, c01Value, c11Value, c12Value, c02Value, c12Value, c22Value])
    
    rovio_fea_disID = td.getColIDs(rovio_fea_dis)
    if rovio_fea_disID != None:
        disField, = [x for x in msg.fields if x.name == 'd']
        for i in np.arange(nFeatures):
            disValue, = struct.unpack('f', msg.data[i*step+disField.offset:i*step+disField.offset+4])
            td.d[ind, rovio_fea_disID[i]] = disValue
    rovio_fea_disCovID = td.getColIDs(rovio_fea_disCov)
    if rovio_fea_disCovID != None:
        disCovField, = [x for x in msg.fields if x.name == 'c_d']
        for i in np.arange(nFeatures):
            disCovValue, = struct.unpack('f', msg.data[i*step+disCovField.offset:i*step+disCovField.offset+4])
            td.d[ind, rovio_fea_disCovID[i]] = disCovValue

def addImuWithCovariance(td, ind, msg, gyr = None, acc = None, gyrCov = None, accCov = None):
    gyrID = td.getColIDs(gyr)
    if gyrID != None:
        td.d[ind, gyrID] = np.array([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]);
    accID = td.getColIDs(acc)
    if accID != None:
        td.d[ind, accID] = np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]);
    gyrCovID = td.getColIDs(gyrCov)
    if gyrCovID != None:
        td.d[ind, gyrCovID] = np.array([msg.angular_velocity_covariance[0],msg.angular_velocity_covariance[1],msg.angular_velocity_covariance[2],
                                                       msg.angular_velocity_covariance[3],msg.angular_velocity_covariance[4],msg.angular_velocity_covariance[5],
                                                       msg.angular_velocity_covariance[6],msg.angular_velocity_covariance[7],msg.angular_velocity_covariance[8]]);
    accCovID = td.getColIDs(accCov)
    if accCovID != None:
        td.d[ind, accCovID] = np.array([msg.linear_acceleration_covariance[0],msg.linear_acceleration_covariance[1],msg.linear_acceleration_covariance[2],
                                                       msg.linear_acceleration_covariance[3],msg.linear_acceleration_covariance[4],msg.linear_acceleration_covariance[5],
                                                       msg.linear_acceleration_covariance[6],msg.linear_acceleration_covariance[7],msg.linear_acceleration_covariance[8]]);
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
            
def rosBagCountTopic(bag, topic, t1 = None, t2 = None):
    counter = 0
    for _, msg, _ in bag.read_messages(topics=[topic]):
        firstTime = msg.header.stamp.to_sec()
        break
    for _, msg, _ in bag.read_messages(topics=[topic]):
        if t1 and msg.header.stamp.to_sec() < firstTime + t1:
            continue
        if t2 and msg.header.stamp.to_sec() > firstTime + t2:
            break
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
        j = 0
        for top, msg, t in bag.read_messages(topics=[topic]):
            while ((j <= td.last) and (td.d[j,td.timeID] < msg.header.stamp.to_sec())):
                j += 1
            if((j <= td.last) and (td.d[j,td.timeID] == msg.header.stamp.to_sec())):
                addTransformStamped(td, j, msg, posID, attID)
            elif(j == td.last + 1):
                td.append()
                td.d[td.last, td.timeID] = msg.header.stamp.to_sec();
                addTransformStamped(td, td.last, msg, posID, attID)
            else:
                print('Could not merge entry into already existing Timed Data');
    bag.close()

def rosBagLoadOdometry(filename, topic, td, posID = None, attID = None, velID = None, rorID = None, posCovID = None, attCovID = None, velCovID = None, rorCovID = None, t1 = None, t2 = None, pruneDuplicate = None):
    bag = rb.Bag(filename)
    count = rosBagCountTopic(bag,topic,t1,t2)
    print("loading " + filename + ", found " + str(count) +" "+ topic +" entries")
    for _, msg, _ in bag.read_messages(topics=[topic]):
        firstTime = msg.header.stamp.to_sec()
        break
    lastMsg = None
    if( td.last == (-1) ):
        for _, msg, _ in bag.read_messages(topics=[topic]):
            if t1 and msg.header.stamp.to_sec() < firstTime + t1:
                continue
            if t2 and msg.header.stamp.to_sec() > firstTime + t2:
                break
            if (pruneDuplicate and lastMsg and lastMsg.pose == msg.pose):
                continue
            lastMsg = msg
            td.append()
            td.d[td.last, td.timeID] = msg.header.stamp.to_sec();
            addOdometry(td, td.last, msg, posID, attID, velID, rorID, posCovID, attCovID, velCovID, rorCovID)
    else:
        print('Implement functionality when timedata is not empty');
    bag.close()

def rosBagLoadPoseWithCovariance(filename, topic, td, posID = None, attID = None, posCovID = None, attCovID = None):
    bag = rb.Bag(filename)
    count = rosBagCountTopic(bag,topic)
    print("loading " + filename + ", found " + str(count) +" "+ topic +" entries")
    notEmpty = (td.last > -1)
    c = 0
    if((not notEmpty) or td.last+1 == count):
        for top, msg, t in bag.read_messages(topics=[topic]):
            if(not notEmpty):
                td.append()
                c = td.last
                td.d[c, td.timeID] = msg.header.stamp.to_sec();
            addPoseWithCovariance(td, c, msg, posID, attID, posCovID, attCovID)
            c += 1
    else:
        print('Implement functionality when timedata is not empty');
    bag.close()

def rosBagLoadTwistWithCovariance(filename, topic, td, velID = None, rorID = None, velCovID = None, rorCovID = None):
    bag = rb.Bag(filename)
    count = rosBagCountTopic(bag,topic)
    print("loading " + filename + ", found " + str(count) +" "+ topic +" entries")
    notEmpty = (td.last > -1)
    c = 0
    if((not notEmpty) or td.last+1 == count):
        for top, msg, t in bag.read_messages(topics=[topic]):
            if(not notEmpty):
                td.append()
                c = td.last
                td.d[c, td.timeID] = msg.header.stamp.to_sec();
            addTwistWithCovariance(td, c, msg, velID, rorID, velCovID, rorCovID)
            c += 1
    else:
        print('Implement functionality when timedata is not empty');
    bag.close()

def rosBagLoadImuWithCovariance(filename, topic, td, gyr = None, acc = None, gyrCov = None, accCov = None):
    bag = rb.Bag(filename)
    count = rosBagCountTopic(bag,topic)
    print("loading " + filename + ", found " + str(count) +" "+ topic +" entries")
    notEmpty = (td.last > -1)
    c = 0
    if((not notEmpty) or td.last+1 == count):
        for top, msg, t in bag.read_messages(topics=[topic]):
            if(not notEmpty):
                td.append()
                c = td.last
                td.d[c, td.timeID] = msg.header.stamp.to_sec();
            addImuWithCovariance(td, c, msg, gyr, acc, gyrCov, accCov)
            c += 1
    else:
        print('Implement functionality when timedata is not empty');
    bag.close()

def rosBagLoadRobocentricPointCloud(filename, topic, td, rovio_fea_idx, rovio_fea_pos, rovio_fea_cov = None, rovio_fea_dis = None, rovio_fea_disCov = None):
    bag = rb.Bag(filename)
    count = rosBagCountTopic(bag,topic)
    print("loading " + filename + ", found " + str(count) +" "+ topic +" entries")
    if( td.last == (-1) ):
        for top, msg, t in bag.read_messages(topics=[topic]):
            td.append()
            td.d[td.last, td.timeID] = msg.header.stamp.to_sec();
            addRobocentricPointCloud(td, td.last, msg, rovio_fea_idx, rovio_fea_pos, rovio_fea_cov,rovio_fea_dis,rovio_fea_disCov)
    else:
        j = 0
        for top, msg, t in bag.read_messages(topics=[topic]):
            while ((j <= td.last) and (td.d[j,td.timeID] < msg.header.stamp.to_sec())):
                j += 1
            if((j <= td.last) and (td.d[j,td.timeID] == msg.header.stamp.to_sec())):
                addRobocentricPointCloud(td, j, msg, rovio_fea_idx, rovio_fea_pos, rovio_fea_cov,rovio_fea_dis,rovio_fea_disCov)
            else:
                print('Could not merge entry into already existing Timed Data');
    bag.close()

def rosBagLoadTimestampsOnly(filename, topic, td, t1 = None, t2 = None):
    bag = rb.Bag(filename)
    count = rosBagCountTopic(bag,topic,t1,t2)
    print("loading " + filename + ", found " + str(count) +" "+ topic +" entries")
    for _, msg, _ in bag.read_messages(topics=[topic]):
        firstTime = msg.header.stamp.to_sec()
        break
    if( td.last == (-1) ):
        for _, msg, _ in bag.read_messages(topics=[topic]):
            if t1 and msg.header.stamp.to_sec() < firstTime + t1:
                continue
            if t2 and msg.header.stamp.to_sec() > firstTime + t2:
                break
            td.append()
            td.d[td.last, td.timeID] = msg.header.stamp.to_sec();
    else:
        print('Implement functionality when timedata is not empty');
    bag.close()
    
    
    
    
    
    
    