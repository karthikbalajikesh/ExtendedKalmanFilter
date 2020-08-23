#include "kalman_filter.h"
#include "math.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
   x_ = F_ * x_;
   // make sure no zero in px and py
   if(abs(x_(0))<0.001){
     x_(0) = 0.001;
   }
   if(abs(x_(1))<0.001){
     x_(1) = 0.001;
   }

   P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  ///////////////////////////////////////////////////////
  // y = z - H*x
  Eigen::VectorXd y = z - (H_ * x_);
  Eigen::MatrixXd S = (H_ * P_ * H_.transpose()) + R_;
  if(fabs(S.determinant()) <0.0001){
    // put state equal to measurements
    x_(0) = z(0);
    x_(1) = z(1);
  }else{
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(4,4) - K*H_)*P_;
  }
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //////////////////////////////////////////////////////
  // y = z - h(x)
  Eigen::VectorXd y = updateY(z);
  Eigen::MatrixXd S = (H_ * P_ * H_.transpose()) + R_;

  if(fabs(S.determinant()) <0.0001){
    // put state equal to measurements
    x_(0) = z(0) * cos(z(1));
    x_(1) = z(0) * sin(z(1));
    x_(2) = z(2) * cos(z(1));
    x_(3) = z(2) * sin(z(1));

  }else{
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(4,4) - K*H_)*P_;
  }

}

Eigen::VectorXd KalmanFilter::updateY(Eigen::VectorXd z){

  Eigen::VectorXd y(3);
  // compute sqrt(x^2 + y^2)
  float temp = sqrt(pow(x_(0),2) + pow(x_(1),2));
  y(0) = z(0) - temp;
  y(1) = z(1) - atan2(x_(1),x_(0));



  if(temp<0.0001){
    y(2) = 0;
  } else{
    y(2) = (x_(0) * x_(2) + x_(1)*x_(3))/temp;
  }

  return y;
}
