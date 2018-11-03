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
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  if (estimations.size() == 0)
  {
  	cout << "Error! The estimations should not be zero!" << endl;
  }
  if (estimations.size() != ground_truth.size())
  {
  	cout << "Error! The estimation size should equal ground truth size." << endl;
  }
  VectorXd temp(4);
  for(int i=0; i < estimations.size(); ++i)
  {
  	temp = estimations[i] - ground_truth[i];
  	temp = temp.array()*temp.array();
  	rmse += temp;
  }
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3,4);
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	if ((px == py) && (px == 0))
	{
		cout << "Error! Divided by zero!" << endl;
		return Hj;
	}
	//double sqr;
	//sqr = pow((px*px + py*py), 0.5);
  double c1 = px*px + py*py;
  double c2 = sqrt(c1);
  double c3 = c1 * c2;
	Hj << px/c2, py/c2, 0, 0,
	      -py/c1, px/c1, 0, 0,
	      py*(vx*py-vy*px)/c3, px*(vy*px-vx*py)/c3, px/c2, py/c2;

	return Hj;
}
