# Pedestrian Tracking by Sensor Fusion of RADAR and LIDAR data using an Extended Kalman Filter

Udacity Self-Driving Car Nanodegree project. A data file with inputs and timestamp is provided and ground truth positions are given. The task of the student is to fuse the data from the LIDAR and RADAR using a bayesian filter, to estimate the pose of the pedestrian, with a motion model to provide the prior and a suitable measurement model to estimate the likelihood. A starter code repo is given at [Udacity Github Page](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) with TODO sections to be completed. 


## Running the code
First clone this repository. To make sure that all the dependencies are installed, please run 

`./install-ubuntu.sh`

Then to run the code,

1. `mkdir build && cd build`

2. `cmake ..`

3. `make`

4. `./ExtendedKF`



## Filter Explanation

We have two sensors - A LIDAR and a RADAR. We will need to estimate the position based on the sensor data and knowledge of the vehicle's motion.

### State 

The state vector is composed of 4 elements {px,py,vx,vy}.

### EKF Algorithm

* First we derive a motion model and account for the uncertainty of the motion with Gaussian additive noise with mean 0 and Covariance described by matrix Q. 

* Next we derive a measurement model to predict the observation based on the state predicted by the motion model. The measurement model is dependant on the type of sensor. 

* LIDAR provides direct {px,py} values, to which we add some Gaussian noise. Hence this constitutes a linear measurement model and a linear Kalman Filter is utilized. 

* The RADAR, however measures {Range, Bearing Angle, Range Rate}, due to which the measurement model is a non-linear model with square roots and squares. Hence we utilize the Extended Kalman Filter, which is a linearized Kalman filter that utilizes the Jacobian of the non-linear function to evaluate uncertainties in the model. 

* Based on the sensor data at a given time step, the respective equations are used. 

* The Root Mean Squared Error(RMSE) is calculated to evaluate the performance of the filter. 
