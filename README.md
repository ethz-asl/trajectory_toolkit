# trajectory_toolkit
This is a python tool for analyzing and evaluating trajectory data. The basic structure are numpy array of arbitrary dimension. The first dimension contains the number of timesteps, the second dimension corresponds to the dimension of the data. The following is supported:
* Loading data from rosbag into the arrays
* Reading the data online from a rostopic
* Plotting functionality, including online plotting
* Differentiation of data
* Interpolation of data
* Handling of 3D rotations by means of quaternions
* Time alignment of data
* Alignment of coordinate frames

The example.py files shows the basic functionality of the package. A rosbag containing trajectory data is provided for testing.

Future features:
* Roll/Pitch/Yaw functioanlity for better plotting (including uncertainties)
* Error plot and error measures