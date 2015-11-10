# Importing stuff
import os, sys, inspect
# from pylab import *
# import numpy as np
# cmd_subfolder = "../python"
# if cmd_subfolder not in sys.path:
# 	sys.path.insert(0, cmd_subfolder)
# import listener
# listener = reload(listener)
# from listener import Plotter
# import matplotlib.animation as animation
import matplotlib.pyplot as plt
import rospy
import rosbag

from TimedData import TimedData
import Listeners
from Listeners import TransformStampedListener

rospy.init_node('test', anonymous=True)
rate = rospy.Rate(100)

td1 = TimedData(7)
tsl = TransformStampedListener(td1,"/vicon/firefly_sbx/firefly_sbx",0,3)
 
# matplotlib.pyplot.close('all')

# plt.ion()

# matplotlib.rcParams['text.usetex'] = True
# matplotlib.rcParams['pdf.fonttype'] = 42
# matplotlib.rcParams['ps.fonttype'] = 42



# plotter1 = Plotter(1)
# plotter2 = Plotter(2)
# 
# rospy.Subscriber("/vicon/firefly_sbx/firefly_sbx", TransformStamped, plotter2.callback)

print(rospy.is_shutdown())
while not rospy.is_shutdown():
# 	plotter1.refresh()
# 	plotter2.refresh()
	rate.sleep()


# print((*(td1.d))[0])
plt.plot(td1.t,td1.d)

plt.show()



