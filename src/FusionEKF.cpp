#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized = false;

  previous_timestamp = 0;

  // initializing matrices
  R_laser = MatrixXd(2, 2);
  R_radar = MatrixXd(3, 3);
  H_laser = MatrixXd(2, 4);
  Hj = MatrixXd(3, 4);

  // measurement covariance matrix - laser
  R_laser << 0.0225, 0, 0, 0.0225;

  // measurement covariance matrix - radar
  R_radar << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;

  // measurement function matrix - laser
  H_laser << 1, 0, 0, 0, 0, 1, 0, 0;

  // acceleration noise
  noise_ax = 9;
  noise_ay = 9;

  ekf = KalmanFilter();
  tools = Tools();
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
/*****************************************************************************
 *  Initialization
 ****************************************************************************/
  if (!is_initialized) {
    /**
     * Initialize the state ekf.x with the first measurement.
     * Create the covariance matrix. No need. This is already done in the constructor.
     * Remember: you'll need to convert radar from polar to cartesian
     * coordinates.
     */
    // initial state transition matrix
    MatrixXd F = MatrixXd(4, 4);
    F << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;

    // state convariance matrix
    MatrixXd P = MatrixXd(4, 4);
    P << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000;

    // state vector
    VectorXd x = VectorXd(4);
    x << 0, 0, 0, 0;

    // process covariance matrix
    MatrixXd Q = MatrixXd(4, 4);
    ekf.Init(x, P, F, H_laser, R_laser, Q);

    if (measurement_pack.sensor_type == MeasurementPackage::RADAR) {
      /**
       * Convert radar from polar to cartesian coordinates and initialize state.
       */
      float x = measurement_pack.raw_measurements[0] *
                cos(measurement_pack.raw_measurements[1]);
      float y = measurement_pack.raw_measurements[0] *
                sin(measurement_pack.raw_measurements[1]);
      ekf.x << x, y, 0, 0;
    } else if (measurement_pack.sensor_type == MeasurementPackage::LASER) {
      /**
       * Initialize state.
       */
      ekf.x << measurement_pack.raw_measurements[0],
          measurement_pack.raw_measurements[1], 0, 0;
      if (fabs(ekf.x(0)) < EPS && fabs(ekf.x(1)) < EPS) {
        exit(0);
        ekf.x(0) = EPS;
        ekf.x(1) = EPS;
      }
    }
    previous_timestamp = measurement_pack.timestamp;

    // done initializing, no need to predict or update
    is_initialized = true;

    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * - Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp - previous_timestamp) / 1e6;
  previous_timestamp = measurement_pack.timestamp;

  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;

  // modify the F matrix so that the time is integrated
  ekf.F(0, 2) = dt;
  ekf.F(1, 3) = dt;

  // update the process covariance matrix Q
  ekf.Q(0, 0) = dt4 / 4 * noise_ax;
  ekf.Q(0, 1) = 0.f;
  ekf.Q(0, 2) = dt3 / 2 * noise_ax;
  ekf.Q(0, 3) = 0.f;
  ekf.Q(1, 0) = 0.f;
  ekf.Q(1, 1) = dt4 / 4 * noise_ay;
  ekf.Q(1, 2) = 0.f;
  ekf.Q(1, 3) = dt3 / 2 * noise_ay;
  ekf.Q(2, 0) = dt3 / 2 * noise_ax;
  ekf.Q(2, 1) = 0.f;
  ekf.Q(2, 2) = dt2 * noise_ax;
  ekf.Q(2, 3) = 0.f;
  ekf.Q(3, 0) = 0.f;
  ekf.Q(3, 1) = dt3 / 2 * noise_ay;
  ekf.Q(3, 2) = 0.f;
  ekf.Q(3, 3) = dt2 * noise_ay;

  ekf.Predict(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   * Use the sensor type to perform the update step.
   * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type == MeasurementPackage::RADAR) {
    // Radar updates
    ekf.R = R_radar;
    ekf.H = Hj;
    VectorXd z = VectorXd(3);
    z << measurement_pack.raw_measurements[0],
        measurement_pack.raw_measurements[1],
        measurement_pack.raw_measurements[2];
    tools.CalculateJacobian(ekf.H, ekf.x);
    ekf.UpdateEKF(z);
  } else {
    // Laser updates
    ekf.R = R_laser;
    ekf.H = H_laser;
    VectorXd z = VectorXd(2);
    z << measurement_pack.raw_measurements[0],
        measurement_pack.raw_measurements[1];
    ekf.Update(z);
  }

  // print the output
  cout << "x = " << ekf.x << endl;
  cout << "P = " << ekf.P << endl;
}
