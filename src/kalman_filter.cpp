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
  P_ = F_ * P_ * F_.transpose() + Q_;
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
  // Calculate h(x) from predicted state to predicted radar measurements
  VectorXd h = VectorXd(z.size());
  h(0) = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  if(fabs(x_(0))> 0.001 && fabs(x_(1)) > 0.001)
	 h(1) = atan2(x_(1), x_(0));
  else
	 h(1) = 0;
  h(2) = (x_(0)*x_(2)+x_(1)*x_(3))/h(0);
    
  VectorXd y  = z - h;
  // Normalise phi difference to be between -pi and pi
  y(1) = atan2(sin(y(1)),cos(y(1)));
  
  MatrixXd S  = H_radar_ * P_ * H_radar_.transpose() + R_radar_;
  MatrixXd K  = P_ * H_radar_.transpose() * S.inverse();
  
  // New state
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_radar_) * P_;
  
}
