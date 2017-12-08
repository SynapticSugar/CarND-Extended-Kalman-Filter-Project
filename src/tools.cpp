#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;



Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
    /**
     * Calculate the RMSE here.
     */
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    if (estimations.size() == 0)
    {
        cout << "the estimation vector size should not be zero";
        return rmse;
    }

    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size())
    {
        cout << "the estimation vector size should equal ground truth vector size";
        return rmse;
    }

    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i) {
        VectorXd a(4);
        a = estimations[i] - ground_truth[i];
        rmse = rmse.array() + a.array() * a.array();
    }

    //calculate the mean
    rmse = rmse.array()/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

void Tools::CalculateJacobian(MatrixXd& Hj, const VectorXd& x_state)
{
    /**
     * Calculate a Jacobian here.
     */
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    if (fabs(px) < EPS && fabs(py) < EPS)
    {
        px = EPS;
        py = EPS;
    }
    
    // pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px+py*py;

    // check division by zero
    if( fabs(c1) < EPS )
    {
        cout << "CalculateJacobian () - Warning - Division by Zero" << endl;
        c1 = EPS;
    }

    float c2 = sqrt(c1);
    float c3 = (c1*c2);

    // compute the Jacobian matrix
    Hj(0,0) = px/c2;
    Hj(0,1) = py/c2;
    Hj(0,2) = 0;
    Hj(0,3) = 0;
    Hj(1,0) = -py/c1;
    Hj(1,1) = px/c1;
    Hj(1,2) = 0;
    Hj(1,3) = 0;
    Hj(2,0) = py*(vx*py - vy*px)/c3;
    Hj(2,1) = px*(px*vy - py*vx)/c3;
    Hj(2,2) = px/c2;
    Hj(2,3) = py/c2;
}
