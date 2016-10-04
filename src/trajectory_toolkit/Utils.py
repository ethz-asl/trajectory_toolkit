import numpy as np
import rospy
import hashlib
from pylab import *
import os, sys, inspect

colors = {'lightred': '#ffaa88', 'lightgreen': '#88ffaa', 'lightblue': '#88aaff', 'lightgray': '#cccccc',
          'red': '#dd4422', 'green': '#22dd44', 'blue': '#2244dd', 'gray': '#888888',
          'darkred': '#992200', 'darkgreen': '#009922', 'darkblue': '#002299', 'darkgray': '#444444'}

def norm(v): #TESTED
    return np.sum(v**2,axis=-1)**(1./2)

def skew(v):
    if v.ndim == 1:
        S = np.zeros([9])
    else:
        S = np.zeros([np.shape(v)[0],9])
    
    S.T[1,] = -v.T[2,]
    S.T[2,] =  v.T[1,]
    S.T[3,] =  v.T[2,]
    S.T[5,] = -v.T[0,]
    S.T[6,] = -v.T[1,]
    S.T[7,] =  v.T[0,]
    return S

def matrixPower(M,a):
    if M.ndim == 1:
        n = len(M)**(0.5)
        if n*n == len(M):
            m = np.resize(M,(n,n))
            return np.resize(np.dot(m,m),9)
        else:
            print('Not square matrix')
    else:
        n = np.shape(M)[1]**0.5
        if n*n == np.shape(M)[1]:
            Mout = np.empty_like(M)
            for i in xrange(0,np.shape(M)[0]):
                m = np.resize(M[i,:],(n,n))
                Mout[i,:] = np.resize(np.dot(m,m),9)
            return Mout
        else:
            print('Not square matrix')


def createIncrementalIds(startId,n):
    ids = range(startId[0],startId[0]+n)
    startId[0] = startId[0]+n
    return ids

def getLen(ids):
    if(isinstance(ids, int)):
        return 1
    else:
        return len(ids)

def hashfile(path, blocksize = 65536):
    afile = open(path, 'rb')
    hasher = hashlib.md5()
    buf = afile.read(blocksize)
    while len(buf) > 0:
        hasher.update(buf)
        buf = afile.read(blocksize)
    afile.close()
    return hasher.hexdigest()

def findDup(parentFolder, previousFile):
    search_hash = hashfile(previousFile)
    for dirName, subdirs, fileList in os.walk(parentFolder):
        for filename in fileList:
            path = os.path.join(dirName, filename)
            file_hash = hashfile(path)
            if(search_hash == file_hash and path != previousFile):
                return path
    return None

def runBagWithInfo(rovioExec, bag, info, camera0, camera1, imu_t, cam0_t, cam1_t, r_odom, r_ext, r_bias, r_pcl, path, out, checkForSameOutName = False, checkForSameInfo = False):
    previous_execution = findDup(path,info)
    if(checkForSameOutName and os.path.isfile(path + out + '.bag')):
        print('Found previous execution of bag with same output name')
        return path + out + '.bag'
    if(checkForSameInfo and previous_execution != None):
        print('Found previous execution of bag with same info file')
        return previous_execution[:-4] + 'bag'
    rospy.set_param('/rovio/camera0_config', camera0)
    rospy.set_param('/rovio/camera1_config', camera1)
    rospy.set_param('/rovio/rosbag_filename', bag)
    rospy.set_param('/rovio/imu_topic_name', imu_t)
    rospy.set_param('/rovio/cam0_topic_name', cam0_t)
    rospy.set_param('/rovio/cam1_topic_name', cam1_t)
    rospy.set_param('/rovio/record_odometry', r_odom)
    rospy.set_param('/rovio/record_extrinsics', r_ext)
    rospy.set_param('/rovio/record_imu_bias', r_bias)
    rospy.set_param('/rovio/record_pcl', r_pcl)
    rospy.set_param('/rovio/filename_out', path + out)
    rospy.set_param('/rovio/filter_config', info)
    os.system(rovioExec)
    return path + out + '.bag'

def poseToInfoOutput(q,r,q_string,r_string):
    out = q_string + 'x' + '\t' + str(q[1]) + '\n'
    out = out + q_string + 'y' + '\t' + str(q[2]) + '\n'
    out = out + q_string + 'z' + '\t' + str(q[3]) + '\n'
    out = out + q_string + 'w' + '\t' + str(q[0]) + '\n'
    out = out + r_string + 'x' + '\t' + str(r[0]) + '\n'
    out = out + r_string + 'y' + '\t' + str(r[1]) + '\n'
    out = out + r_string + 'z' + '\t' + str(r[2]) + '\n'
    print(out)
    
def createFolderIfMissing(path):
    try:
        os.stat(path)
    except:
        os.makedirs(path)

def plotBoxPlot(figureID, data, title, labels, xlabel, ylabel, faceColor, legends = None):
    plt.ion()            
    if(figureID < 0):
        figure()
    elif(figureID >= 0):
        figure(figureID)
    N = len(data)
    M = len(data[0])
    if (M != len(labels)):
        print("Error: data dimension must fit labels")
        return
    if (legends != None and N != len(legends)):
        print("Error: data dimension must fit legends")
        return
    p = np.arange(M)
    for i in range(N):
        bp = boxplot(data[i], sym='', positions = p+i*(1.0/(N+1)), widths = (1.0/(N+2)), patch_artist=True)
        plt.setp(bp['boxes'], color=faceColor[i])
        plt.setp(bp['boxes'], edgecolor='black')
        plt.setp(bp['whiskers'], color='black')
        plt.setp(bp['medians'], color='black')
        if (legends != None):
            plot([0.0],[0.0],faceColor[i], label=legends[i])
    bp = boxplot(data[N/2], sym='', positions = p+N/2*(1.0/(N+1)), widths = (1.0/(N+2)), patch_artist=True)
    plt.title(title)
    plt.setp(bp['boxes'], color=faceColor[N/2])
    plt.setp(bp['boxes'], edgecolor='black')
    plt.setp(bp['whiskers'], color='black')
    plt.setp(bp['medians'], color='black')
    if (legends != None):
        plt.legend(loc=2)
    ax = axes()
    ax.set_xticklabels(labels)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    plt.draw()

def toPiRange(x):
    xOut = (x + math.pi)%(2*math.pi) - math.pi
    return xOut 
    