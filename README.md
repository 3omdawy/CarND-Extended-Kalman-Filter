# CarND-Extended-Kalman-Filter
Self-Driving Car Engineer Nanodegree Program

---
## Description
* This is the 1st project in term 2 of Udacity self-driving cars nano-degree. It implements EKF in C++ to track a bicycle given lidar and radar sensors


## Dependencies

* cmake >= 3.5
 * Used installer: cmake-3.7.2-win64-x64.msi
* make >= 4.1
  * Used installer: make-3.81.exe
* gcc/g++ >= 5.4
  * Used installer: mingw-get-setup.exe

## Basic Build Instructions
Once you have this repository on your machine, `cd` into the repository's root directory and run the following commands from the command line:
```
mkdir build && cd build
cmake .. && make
ExtendedKF (path_to_input).txt (path_to_output).txt
    - eg. `ExtendedKF ../data/obj_pose-laser-radar-synthetic-input.txt output.txt`
```
**NOTE**
> If you encounter any problems, copy "vcvars32.bat" to build directory and run the command `vcvars32` to set environment variables

> If make command does not work try: `cmake .. -G "Unix Makefiles" && make`

> You can find some sample inputs in 'data/'.

## Algorithm
```
	For each measurement in the file	
		If this is the first measurement
			If it is from Radar
				Convert from polar coordinates to cartesian coordinates
			End If
			Initialize measurements
		Else
			If current timestamp is different from previous timestamp
				Calculate F and Q matrices based on delta t (difference in timestamps)
				Predict the current state
			End If
			If the measurement is from Radar
				Calculate Jacobian matrix
				Use R matrix of radar
				Perform measurement update using EKF equation
			Else
				Use R matrix of laser
				Perform measurement update using KF equation
			End If
		End If
		Print current x and P
	End For
```

## Testing
* Against dataset 1:
	- Accuracy - RMSE = 0.065 - 0.062 - 0.534 - 0.58
* Against dataset 2:
	- Accuracy - RMSE = 0.186 - 0.19 - 0.474 - 0.827

* Log files are stored in the folder (output)