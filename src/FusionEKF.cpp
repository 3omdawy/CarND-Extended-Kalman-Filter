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

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
	//state covariance matrix P
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.Q_ = MatrixXd(4, 4);
	
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;
	//measurement matrix
	H_laser_ << 1, 0, 0, 0,
		0, 1, 0, 0;
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
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
	  // x position = Rho * cos ( Phi )
	  // y position = - Rho * sin ( Phi )
	  // vx = Rho_dot * cos ( Phi )
	  // vy = -Rho_dot * sin ( Phi )
	  ekf_.x_ << 	measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]),
						-measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]),
						measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]),
						-measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]);
    //cout << "1st Measurement from radar" << endl;						
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
	  //set the state with the initial location and zero velocity
	  ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    //cout << "1st Measurement from laser" << endl;
    }
	previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
   //Predict only if the current timestamp is != last timestamp
   if (measurement_pack.timestamp_ != previous_timestamp_)
   {
   double noise_ax = 9;
   double noise_ay = 9;
   
   double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
   previous_timestamp_ = measurement_pack.timestamp_;

   //1. Modify the F matrix so that the time is integrated
	ekf_.F_ << 1, 0, dt, 0,
		0, 1, 0, dt,
		0, 0, 1, 0,
		0, 0, 0, 1;
	//2. Set the process covariance matrix Q
	ekf_.Q_ << (dt * dt * dt * dt * noise_ax / 4), 0, (dt * dt * dt * noise_ax / 2), 0,
		0, (dt * dt * dt * dt * noise_ay / 4), 0, (dt * dt * dt * noise_ay / 2),
		(dt * dt * dt * noise_ax / 2), 0, (dt * dt * noise_ax), 0,
		0, (dt * dt * dt * noise_ay / 2), 0, (dt * dt * noise_ay);
		
  ekf_.Predict();
    //cout << "Prediction is OK" << endl;
   }

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	ekf_.R_ = R_radar_;
	ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    //cout << "Measurement from radar is OK" << endl;
  } else {
    // Laser updates
	ekf_.R_ = R_laser_;
	ekf_.H_ = H_laser_;
	ekf_.Update(measurement_pack.raw_measurements_);
    //cout << "Measurement from laser is OK" << endl;
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
