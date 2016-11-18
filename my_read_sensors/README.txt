my_read_sensors is written for HW 5 of MAE 144.

This program uses the Beaglebone Black and the Robotics Cape to estimate the 
angle (in radians) of the Beaglebone around the X-axis of the cape, calibrated 
such that 0 radians corresponds to the y-axis pointing up and -pi/2 radians 
when the z-axis is pointing up.

Two angle estimates are generated. The first estimate uses the ratio between 
the y- and z- accelerations while the second estimate integrates the x-axis
gyroscope data.
