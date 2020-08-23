#include "tools.h"
#include "math.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // check for size error
  if(estimations.size() != ground_truth.size()){
    std::cout<<"CalculateRMSE() Error - Sizes Mismatch"<<std::endl;
    return;
  }
  Eigen::VectorXd RMSE(4);
  RMSE<<0,0,0,0;

  size_t N = estimations.size();
  for(size_t i = 0; i<N; i++){
    Eigen::VectorXd temp(4);
    temp =estimations[i] - ground_truth[i];
    RMSE += temp.array() * temp.array();
  }
  RMSE = RMSE/N;
  RMSE = RMSE.array().sqrt();

  return RMSE;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /////////////////////////////////////////////////////////
  Eigen::MatrixXd Jacobian(3,4);
  float x = x_state(0), y = x_state(1);
  float vx = x_state(2), vy = x_state(3);
  float temp = sqrt(x*x + y*y);
  if(fabs(temp) < 0.0001){
    std::cout<<"CalculateJacobian() Error - Division by Zero"<<std::endl;
    return Jacobian;
  }
  Jacobian << x/temp, y/temp,0,0,
              -y/(pow(temp,2)),x/(pow(temp,2)),0,0,
              y*(y*vx - x*vy)/(pow(temp,3)),x*(x*vy - y*vx)/(pow(temp,3)),
                        x/temp,y/temp;

  return Jacobian;
}
