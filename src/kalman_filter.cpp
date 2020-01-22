#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

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
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_ * x_;
  
  UpdateAfterY(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  bool test = false;
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  //recover state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
    
  // Equations for h_func below
  float rho = sqrt(px * px + py * py);
  float phi = atan2(py,px);
  float rho_dot = (px*vx+py*vy)/rho;
  //check division by zero
  if(test && rho < .00001) { 
    cout << "px: " << px << "; py: " << py << "; vx: " << vx << "; vy: " << vy << "; rho: "<<rho<<"; phi: "<<phi<<"; rho_dot: "<<rho_dot<<endl;
    rho = 0.00001;
    rho_dot = 0;
  }
    
  //Feed in equations above
  VectorXd H_func(3);
  H_func << rho, phi, rho_dot;  
  VectorXd y = z - H_func;
  // Normalize the angle
  while (y(1)>M_PI) {
    y(1) -= 2 * M_PI;
  }
  while (y(1)<-M_PI) {
    y(1) += 2 * M_PI;
  }
  
  UpdateAfterY(y);
}

// The common part of both Update functions
void KalmanFilter::UpdateAfterY(const VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K = P_ * Ht * S.inverse();
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}