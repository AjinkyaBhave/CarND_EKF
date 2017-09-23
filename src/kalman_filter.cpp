#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &Q_in, 
								MatrixXd &H_laser_in, MatrixXd &H_radar_in, MatrixXd &R_laser_in, MatrixXd &R_radar_in ) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;
  H_laser_ = H_laser_in;
  H_radar_ = H_radar_in;
  R_laser_ = R_laser_in;
  R_radar_ = R_radar_in;
  I_ = MatrixXd::Identity(x_.size(), x_.size());

  
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P  = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y  = z - H_laser_ * x_;
  MatrixXd S  = H_laser_ * P_ * H_laser_.transpose() + R_laser_;
  MatrixXd K  = P_ * H_laser_.transpose() * S.inverse();
  
  //new state
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
}
