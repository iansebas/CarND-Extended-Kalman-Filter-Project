#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {



	//////////////
	// Method 1 //
	//////////////

	// assert(estimations.size()==ground_truth.size());

	// int size_m = estimations.size();

	// VectorXd RMSE_vec(size_m);

	// for (int i = 0; i < size_m; ++i){

	// 	double rmse = 0;
	// 	assert(estimations(i).size()==ground_truth(i).size());
	// 	double size_m = estimations(i).size();
	// 	for (int j = 0; j < estimations(i).size(); ++j){
	// 		rmse = (ground_truth(i)(j) - estimations(i)(j))*(ground_truth(i)(j) - estimations(i)(j))
	// 	}

	// 	rmse = sqrt(rmse/size_m)

	// 	RMSE_vec(i) = rmse; 
	// }

	// return RMSE_vec;

	//////////////
	// Method 2 //
	//////////////

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */


	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	//check division by zero
	if(fabs(c1) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;

}


VectorXd Tools::radar_mapping(const VectorXd& x_state) {

	VectorXd hx_state(3);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	float c1 = sqrt(px*px+py*py);
	float c2 = atan2(py,px);

	assert(abs(c2)<3.1415);

	//check division by zero
	if(fabs(c2) < 0.0001){
		cout << "Radar_Mapping () - Error - Division by Zero" << endl;
		return hx_state;
	}

	float c3 = (px*vx+py*vy)/c1;

	hx_state <<	c1,c2,c3;

	return hx_state;
}

void Tools::normalize_angle(VectorXd& y) {

	float theta = y(1);

	if (theta>M_PI){

		while (theta>M_PI){
			theta = theta - 2*M_PI;
		}



	} else if (theta<-M_PI) {

		while (theta<-M_PI){
			theta = theta + 2*M_PI;
		}
		

	}

	assert(abs(theta)<3.1415);

	y(1) = theta;

}

