#ifndef EKFCTRV_EKF_H
#define EKFCTRV_EKF_H


#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <time.h>
#include "kf.h"

class EKF_CTRV {
	/*CTRV模型，x、y方向的速度由速度以及方向决定，可以跟踪前车的状态
	* x,y,
	* v 追踪车辆相对于自车的速度,
	* θ 偏航角，是追踪的目标车辆在当前车辆坐标系下与Y轴（即车前方向）的夹角),在车辆坐标系下的方位角
	* ω 偏航角速度
	*/

public:

	EKF_CTRV();

	virtual ~EKF_CTRV();


	void ProcessMeasurement(const MeasurementPackage &meas_package);

	/*状态转移函数*/
	void StateTransition(double delta_t);
	/*计算噪声协方差矩阵*/
	void ProcessQMatrix(double delta_t);
	/*计算状态转移矩阵的雅克比矩阵*/
	void ProcessJAMatrix(double delta_t);

	/*计算毫米波雷达映射矩阵对应的雅克比矩阵*/
	Eigen::VectorXd ProcessHJMatrix();

	void Update(const Eigen::VectorXd &z);


	void UpdateEKF(const Eigen::VectorXd &z);
	void Predict(double delta_t);
	void getState(Eigen::VectorXd& x);
	double control_psi(double psi);
private:
	//判断是否被初始化
	bool is_initialized_;

	// 上一数据时间戳
	long long  previous_timestamp_;
	///* 状态向量
	Eigen::VectorXd x_;

	///* 状态x_协方差矩阵、系统的不确定程度
	Eigen::MatrixXd P_;

	// 状态转移函数
	//Eigen::MatrixXd F_;
	Eigen::MatrixXd JA_;

	//处理噪声的协方差矩阵
	Eigen::MatrixXd Q_;

	//测量矩阵
	Eigen::MatrixXd H_;

	// 测量协方差矩阵，表示测量的不确定度，通常由传感器厂家提供
	Eigen::MatrixXd R_;

	Eigen::MatrixXd R_laser_;//激光雷达的误差矩阵
	Eigen::MatrixXd R_radar_;//毫米波雷达的误差矩阵
	Eigen::MatrixXd R_laser_radar_;//毫米波雷达的误差矩阵
	Eigen::MatrixXd H_laser_;//激光雷达映射矩阵

	Eigen::MatrixXd HJ_;//毫米波雷达映射矩阵对应的雅克比矩阵
	Eigen::MatrixXd HJ_LR;//毫米波雷达+激光雷达映射矩阵对应的雅克比矩阵

	// 直线加速度噪声 m/s^2
	double std_a_;

	// 偏航角加速度噪声 rad/s^2
	double std_yawdd_;
};


#endif 
