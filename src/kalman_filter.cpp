#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
//using namespace std;
KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}


void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  std::cout<<"x predicted is "<<x_<<"\n";

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;

  
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
//}

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  VectorXd z_pred2(3);
  z_pred2[0]=sqrt(x_[0]*x_[0]+x_[1]*x_[1]);
//std::cout<<"UpdateEKF"<<sqrt(x_[0]*x_[0]+x_[1]*x_[1])<<"\n";
  z_pred2[1]=atan(x_[1]/x_[0]);
  z_pred2[2]=(x_[0]*x_[2]+x_[1]*x_[3])/z_pred2[0];
  //VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred2;

	//MatrixXd Hj;
	//Hj=tools.CalculateJacobian(x_);	
  double pi=3.1415926535897; 
 if(y[1]<pi)y[1]=y[1]+2*pi;
  if(y[1]>pi)y[1]=y[1]-2*pi;

std::cout<<"UpdateEKF y"<<y<<"\n";
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
//	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
//	P_ = (I - K * H_) * P_;




}
