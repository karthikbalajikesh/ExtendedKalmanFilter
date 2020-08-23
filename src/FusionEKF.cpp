#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include "math.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  ////////////////////////////////////////////////
  H_laser_ = Eigen::MatrixXd(2,4);
  H_laser_ << 1,0,0,0,
             0,1,0,0;


  ///////////////////////////////////////////////
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /////////////////////////////////////////////////////////////
      Eigen::VectorXd measurements = measurement_pack.raw_measurements_;
      initializeWithRADAR(measurements);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ///////////////////////////////////////////////////////////////////
      Eigen::VectorXd measurements = measurement_pack.raw_measurements_;
      initializeWithLIDAR(measurements);
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /*
  * TODO: Update the state transition matrix F according to the new elapsed time.
  * Time is measured in seconds.
  * TODO: Update the process noise cov
Menu
￼SIMULATORariance matrix.
  * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  */
  ///////////////////////////////////////////////////////////
  updateSamplingTime(measurement_pack.timestamp_);
  updateFMatrix();
  updateQMatrix();
  //////////////////////////////////////////////////////////
  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // This is the RADAR update (nonlinear)
    ekf_.R_ = this->R_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Lidar update (linear)
    ekf_.H_ = this->H_laser_;
    ekf_.R_ = this->R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}

// Private Functions:

// 1. Function to initialize state with RADAR
void FusionEKF::initializeWithRADAR(Eigen::VectorXd &measurements){
  // 1.  x = R cos phi
  ekf_.x_(0) = measurements(0) * cos(measurements(1));
  // make sure value is not 0
  if(fabs(ekf_.x_(0)) < 0.001){
    ekf_.x_(0) = 0.001;
  }
  // 2.  y = R sin phi
  ekf_.x_(1) = measurements(0) * sin(measurements(1));
  // make sure value is not 0
  if(fabs(ekf_.x_(1)) < 0.001){
    ekf_.x_(1) = 0.001;
  }
  // 3.  vx = Rdot cos phi
  ekf_.x_(2) = measurements(2) * cos(measurements(1));
  // 4.  vy = Rdot sin phi
  ekf_.x_(3) = measurements(2) * sin(measurements(1));

  ekf_.P_ = Eigen::MatrixXd(4,4);
  ekf_.P_ << 1,0,0,0,
             0,1,0,0,
             0,0,10,0,
             0,0,0,10;
  // providing higher uncertainty for velocities
}

// 2. Function to initialize state with Lidar
void FusionEKF::initializeWithLIDAR(Eigen::VectorXd &measurements){
// x= x
  ekf_.x_(0) = measurements(0);
  if(fabs(ekf_.x_(0)) < 0.001){
    ekf_.x_(0) = 0.001;
  }
  // y = y
  ekf_.x_(1) = measurements(1);
  if(fabs(ekf_.x_(1)) < 0.001){
    ekf_.x_(1) = 0.001;
  }
  ekf_.P_ = Eigen::MatrixXd(4,4);
  ekf_.P_ << 1,0,0,0,
             0,1,0,0,
             0,0,1000,0,
             0,0,0,1000;
  // providing very high v uncertainties as lidar is not measuring them
}

// 3. Function to update the sampling time
void FusionEKF::updateSamplingTime(long long timestamp){
  samplingTime = (timestamp - previous_timestamp_)/1000000.0;
  previous_timestamp_ = timestamp;
}

// 4. Function to update the State Transition Matrix F
void FusionEKF::updateFMatrix(){
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1,0,this->samplingTime,0,
             0,1,0,this->samplingTime,
             0,0,1,0,
             0,0,0,1;

}

// 5. Function to update the Process Noise Matrix Q
void FusionEKF::updateQMatrix(){
  ekf_.Q_ = MatrixXd(4,4);
  float dt = this->samplingTime;
  float dt2 = dt * dt;
  int noise_ax = 9;
  int noise_ay = 9;

  ekf_.Q_ << (noise_ax * dt2*dt2)/4,0,(noise_ax *dt*dt2)/2,0,
             0,(noise_ay * dt2*dt2)/4,0,(noise_ay *dt*dt2)/2,
             (noise_ax *dt*dt2)/2,0,noise_ax*dt2,0,
             0,(noise_ay *dt*dt2)/2,0,noise_ay*dt2;
}
