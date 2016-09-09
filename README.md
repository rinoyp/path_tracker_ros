# ROS Packages for Path Tracking

## Description
The following packages are used to navigate
autonomously using Differential GPS

## Required Hardware
To run the path tracking software, the following hardware is
required:
* A differential-drive robot
* A Computer running Ubuntu 14.04
* Swift Piksi GPS Module

## Hardware Setup
1. Plug in one of the Piksi modules into the computer running Ubuntu 14.04
2. Setup the base station in a known location and make sure to set the surveyed
   latitude, longitude and altitude in the settings in the Piksi Console. Then
   set broadcast to true.
3. Attach the PC running Ubuntu 14.04 and the second Piksi to
   the rover.

## Build Instructions
To build the project, the following libraries
are required to be installed:
* ROS Indigo
* Swift Binary Protocol(https://www.github.com/swift-nav/libsbp)
* Numpy
* PySerial
```
git checkout f69cece814fa8643914d36449b00367caec414e4
```

1. Build the Swift Binary Protocol library following
   the instructions in the README file for version
   used. PySerial and numpy will be installed along 
   with it.
2. Build and install the GPS helper library with
   following commands:

	```
	cd GPSLib/
	sudo python setup.py install 
	```
   
3. Build and install the catkin packages using the
   following commands:

	```
	cd pt_ws
	catkin_make install
	```

## How to Run
To start tracking a path run the following command 
from the directory that contains one of the included 
launch files:

```
roslaunch pt_client.launch
```

A differential drive robot will be needed that can accept
the Twist commands which are published on the /pt_cmd
topic that send linear and angular velocity that the vehicle
should drive at to track the path

The following parameters can be set in the
launch file to control the navigation path and
other performance characteristics.

**lookahead_distance (optional)**
Specifies the lookahead distance used by the pure
pursuit controller to stay on the desired path.
Higher lookahead distances lead to slower convergence
to the path while small lookahead distances can lead to
oscillations around the path. Default is 4.5 metres

**max_angular_velocity(optional)**
Sets the max angular velocity used by the pure pursuit
controller to limit the angular velocity so turns are not
too sharp. Default is 1.0 radians

**desired_speed(optional)**
Sets the desired speed to use when following the
path. Default is 1.2 m/s

**input_file(optional)**
This parameter is used to set the file input containing
the points that define the path the rover should travel.
The points are in the form:
```
lat_1, lon_1
lat_2, lon_2
.
.
.
lat_n, lon_n
```

The first point in list defines the starting location.
The second point in the list is the first point to 
travel to. Its assumed that the rover's heading is initially 
pointed towards the first point.

**auto_gen_bounds(Optional, default is false)**
Automatically generates a square of points from the starting
location of the size specified in metres.

**auto_shape(Optional, default is square)**
The shape of the path to generate. Available options
are circle and square.

**auto_bounds_size(Required if auto_gen_bounds set to true)**
The size of the square or radius of circle to generate from the start point

**auto_start_heading(Required if auto_gen_bounds set to true)**
The initial heading to use to create the square path that
the rover's should follow.

**loops(Optional, default is 1)**
The number of loops to travel around the path. If not used,
the rover will travel the path for one loop.

**use_external_heading(Optional, default is false)**
Use a separate sensor to track the heading of the rover. Use this if
a more accurate sensor is available for heading calculations. By default,
the Kalman Filter calculates heading using DGPS 

**heading_sample_t(Optional, default is 1000 milliseconds)**
Set this parameter for the GPSNode to define the rate to
calculate heading measurements. Higher accuracy is obtained with longer
sample periods, although the rate of measurements is reduced.

**piksi_x(optional)**
Sets the X offset of the Piksi receiver from the centre of the Rover. Used
to transform position to centre of rover.

**piksi_y(optional)**
Sets the Y offset of the Piksi receiver from the centre of the Rover. Used
to transform position to centre of rover.
