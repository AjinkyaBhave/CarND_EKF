#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  
  is_initialized_ = false;
  previous_timestamp_ = 0;
  
  // Define state and measurement dimensions
  n_states = 4;
  n_laser_meas = 2;
  n_radar_meas = 3;
  
  // Set the process noise variances
  var_ax = 5;
  var_ay = 5;
  var_vx = 1000;
  var_vy = 1000;
  var_px = 1;
  var_py = 1;

  // initializing matrices
  R_laser_ = MatrixXd(n_laser_meas, n_laser_meas);
  R_radar_ = MatrixXd(n_radar_meas, n_radar_meas);
  H_laser_ = MatrixXd(n_laser_meas, n_states);
  H_radar_ = MatrixXd(n_radar_meas, n_states);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0, 0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  // Initial laser measurement matrix
  H_laser_ << 1, 0, 0, 0,
				 0, 1, 0, 0;	
	
  // Initial laser measurement matrix
  H_radar_ << 0, 0, 0, 0,
				 0, 0, 0, 0,
				 0, 0, 0, 0;		
				 
  // Allocate memory for 4D state vector. We don't know yet the values of the state variables.
  VectorXd x = VectorXd(n_states);

  // Initial state covariance matrix
  MatrixXd P = MatrixXd(n_states, n_states);
  P << var_px, 0, 0, 0,
		 0, var_py, 0, 0,
		 0, 0, var_vx, 0,
		 0, 0, 0, var_vy;

  // Initial state transition matrix
  MatrixXd F = MatrixXd(n_states, n_states);
  F << 1, 0, 1, 0,
	    0, 1, 0, 1,
	    0, 0, 1, 0,
	    0, 0, 0, 1;
	
  // Initial process covariance matrix
  MatrixXd Q = MatrixXd(n_states, n_states);
  Q << 0, 0, 0, 0,
	    0, 0, 0, 0,
	    0, 0, 0, 0,
		 0, 0, 0, 0;
	
	// Initialise KF with matrices
	ekf_.Init(x, P, F, Q, H_laser_, H_radar_, R_laser_, R_radar_);
	
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "First measurement.\n ";
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
      ekf_.x_(0) = measurement_pack.raw_measurements_[0]* cos(measurement_pack.raw_measurements_[1]);
		ekf_.x_(1) = measurement_pack.raw_measurements_[0]* sin(measurement_pack.raw_measurements_[1]);
		// Cannot calculate velocity from range rate directly, so assuming zero.
		ekf_.x_(2) = 0;
		ekf_.x_(3) = 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //set the state with the initial location and zero velocity
	   ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }
	 else{
		 cout << "Invalid sensor type.\n";
		 return;
	 }
	 // Update time stamp for prediction step
	 previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;
    
  // Modify the F matrix so that the time is integrated
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  
  // Set the process covariance matrix Q
  ekf_.Q_(0,0) = (dt4*var_ax)/4;
  ekf_.Q_(0,2) = (dt3*var_ax)/2;
  ekf_.Q_(1,1) = (dt4*var_ay)/4;
  ekf_.Q_(1,3) = (dt3*var_ay)/2;
  ekf_.Q_(2,0) = (dt3*var_ax)/2;
  ekf_.Q_(2,2) = dt2*var_ax;
  ekf_.Q_(3,1) = (dt3*var_ay)/2;
  ekf_.Q_(3,3) = dt2*var_ay;
	
  // Run prediction step
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	 ekf_.H_radar_ = tools.CalculateJacobian(ekf_.x_);
	 ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
  else {
    // Laser updates
	 ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
