#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x = x_in;
  P = P_in;
  F = F_in;
  H = H_in;
  R = R_in;
  Q = Q_in;
}

void KalmanFilter::Predict(const float delta_t) {
  /**
   * predict the state
   */
  F(0, 2) = delta_t;
  F(1, 3) = delta_t;
  x = F * x;
  P = F * P * F.transpose() + Q;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */

  VectorXd y = z - H * x;
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P * Ht + R;
  MatrixXd K = P * Ht * S.inverse();

  // new state estimate
  x = x + (K * y);
  long x_size = x.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P = (I - K * H) * P;
}

// Utility funtion to wrap a radian angle between -pi and +pi
float Wrap2pi(float rval) {
  while (rval > M_PI) {
    rval -= 2 * M_PI;
  }

  while (rval < -M_PI) {
    rval += 2 * M_PI;
  }
  return rval;
}

// Utility funtion to convert a cartesian state into a polar state
VectorXd Cartesian2Polar(VectorXd &x) {
  VectorXd xp = VectorXd(3);
  xp(0) = sqrt(x(0) * x(0) + x(1) * x(1));
  // ensure we do not divide by zero
  if (xp(0) < 1e-6) xp(0) = 1e-6;
  xp(1) = atan2(x(1), x(0));  // atan2 handles the wrap -pi to pi
  xp(2) = (x(0) * x(2) + x(1) * x(3)) / xp(0);
  return xp;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /*
   * update the state by using Extended Kalman Filter equations
   */
  VectorXd xp = Cartesian2Polar(x);
  VectorXd y = z - xp;
  y(1) = Wrap2pi(y(1));  // ensure wrap -pi to pi
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P * Ht + R;
  MatrixXd K = P * Ht * S.inverse();

  // new state estimate
  x = x + (K * y);
  long x_size = x.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P = (I - K * H) * P;
}
