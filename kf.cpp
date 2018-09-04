
#include "kf.h"
#include <iostream>
const float DoublePI = 2 * M_PI;
/**
 * Initializes Unscented Kalman filter
 */
KF::KF() {}

KF::~KF() {}

//初始化
void KF::Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
	Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in)
{
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
}

//预测步骤
void KF::Predict()
{
	//状态预测
	x_ = F_*x_;
	Eigen::MatrixXd Ft = F_.transpose();
	P_ = F_*P_*Ft + Q_;
}

void KF::Update(const Eigen::VectorXd& z)
{
	//基于卡尔曼对状态进行更新
	Eigen::VectorXd z_pred = H_*x_;//状态空间到测量空间的转换
	Eigen::VectorXd y = z - z_pred;//获取测量值与状态值之间的差
	KF::KalmanFilter(y);
}

void KF::UpdateEKF(const Eigen::VectorXd& z)
{
	//基于扩展卡尔曼对状态进行更新
	float px = x_[0];
	float py = x_[1];
	float vx = x_[2];
	float vy = x_[3];
	float c1 = px*px + py*py;

	//状态空间到测量空间的转换
	float rho, phi, rho_dot;
	rho = sqrt(c1);
	if (rho < 0.000001)
		rho = 0.000001;
	phi = atan2(py,px);
	rho_dot = (px*vx + py*vy) / rho;
	Eigen::VectorXd z_pred = Eigen::VectorXd(3);
	z_pred << rho, phi, rho_dot;

	Eigen::VectorXd y = z - z_pred;
	//将角度归一化到【-π，π】
	while (y(1)>M_PI)
		y(1) -= DoublePI;
	while (y(1) < -M_PI)
		y(1) += DoublePI;
	KalmanFilter(y);
}

void KF::KalmanFilter(const Eigen::VectorXd& y)
{
	Eigen::MatrixXd HT = H_.transpose();
	Eigen::MatrixXd S = H_*P_*HT + R_;
	Eigen::MatrixXd Si = S.inverse();
	Eigen::MatrixXd PHT = P_*HT;
	Eigen::MatrixXd K = PHT*Si;

	//状态更新
	x_ = x_ + K*y;
	long x_size = x_.size();
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
	P_ = (I - K*H_)*P_;
}





