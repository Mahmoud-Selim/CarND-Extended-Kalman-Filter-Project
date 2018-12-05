#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */

	VectorXd rmse(4);
	rmse << 0,0,0,0;

    if(estimations.size() == 0 || estimations.size() != ground_truth.size())
    {
        cout<<"Error with input dimensions";
    }
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        VectorXd point_difference = VectorXd(4);
        point_difference = ((estimations[i] - ground_truth[i]).array() * (estimations[i] - ground_truth[i]).array());
		rmse = rmse + point_difference;
	}

	//calculate the mean
    rmse = rmse / estimations.size();

	//calculate the squared root
    rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	//check division by zero
	if(px == 0 && py == 0)
	{
	    cout<<"CalculateJacobian() - Error - Division by Zero" << endl;
	}
	//compute the Jacobian matrix
	else
	{
	    float px2_py2 = px * px + py * py;
	    Hj << px / (sqrt(px2_py2)), py / (sqrt(px2_py2)), 0, 0,
	          -1 * py / px2_py2, px / px2_py2, 0, 0,
	          py * (vx * py - vy * px) / (px2_py2 * sqrt(px2_py2)),
	          px * (vy * px - vx * py) / (px2_py2 * sqrt(px2_py2)),
	          px / (sqrt(px2_py2)), py / (sqrt(px2_py2));
	}

	return Hj;
}
