#include "ekf_ctrv.h"
#include <iostream>


Eigen::MatrixXd CalculateJacobian_ctrv(const Eigen::VectorXd& x_state) {
	/**
	TODO:
	* Calculate a Jacobian here.
	*/
	//std::cout << "Jac 1" << std::endl;
	Eigen::MatrixXd Hj(3, 4);
	//std::cout << "Jac 2" << std::endl;
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	//std::cout << "Jac 3" << std::endl;
	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px + py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	//check division by zero
	if (fabs(c1) < 0.0001){
		std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px / c2), (py / c2), 0, 0,
		-(py / c1), (px / c1), 0, 0,
		py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;

	return Hj;
}
/**
 * Initializes Unscented Kalman filter
 */
EKF_CTRV::EKF_CTRV() {
	is_initialized_ = false;
	previous_timestamp_ = 0;


	H_laser_ = Eigen::MatrixXd(2,4);
	H_laser_ << 1, 0, 0, 0, 
		0, 1, 0, 0;

	// Laser measurement noise standard deviation position1 in m
	float std_laspx_ = 0.15;

	// Laser measurement noise standard deviation position2 in m
	float std_laspy_ = 0.15;
	R_laser_ = Eigen::MatrixXd(2, 2);
    R_laser_ << std_laspx_*std_laspx_, 0,
            0, std_laspy_*std_laspy_;

	// Radar measurement noise standard deviation radius in m
	float std_radrho_ = 0.3;

	// Radar measurement noise standard deviation angle in rad
	float std_radphi_ = 0.03;

	// Radar measurement noise standard deviation radius change in m/s
	float std_radrhodot_ = 0.3;
	R_radar_ = Eigen::MatrixXd(3, 3);
	R_radar_ << std_radrho_*std_radrho_, 0, 0,
            0, std_radphi_*std_radphi_, 0,
			0, 0, std_radrhodot_*std_radrhodot_;
	//状态向量
	ekf_.x_ = Eigen::VectorXd(4);
	//系统状态不确定性
	ekf_.P_ = Eigen::MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;
	ekf_.F_ = Eigen::MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;
	ekf_.Q_ = Eigen::MatrixXd(4, 4);
}

EKF_CTRV::~EKF_CTRV() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void EKF_CTRV::ProcessMeasurement(const MeasurementPackage &meas_package) {

    if (!is_initialized_) {
		/*
		 * 第一次测量时初始化状态向量
		 * 创建协方差矩阵
		 * 对于radar的测量需要将其从极坐标转换为笛卡尔坐标系
		 */
        // first measurement
		ekf_.x_ = Eigen::VectorXd(4);
		ekf_.x_ << 1, 1, 1, 1;

        if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			std::cout << "laser data: "<< std::endl;
			ekf_.x_[0] = meas_package.raw_measurements_[0];
			ekf_.x_[1] = meas_package.raw_measurements_[1];
			ekf_.x_[2] = 0;
			ekf_.x_[3] = 0;
        } else {
			std::cout << "radar data: " << std::endl;
            float rho = meas_package.raw_measurements_[0];
            float phi = meas_package.raw_measurements_[1];
            float rho_dot = meas_package.raw_measurements_[2];
			ekf_.x_[0] = rho * cos(phi);
			ekf_.x_[1] = rho * sin(phi);
			ekf_.x_[2] = rho_dot*cos(phi);
			ekf_.x_[3] = rho_dot*sin(phi);
        }
		previous_timestamp_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }
	/*
	 * 根据时间更新状态转换矩阵F_
	 * 时间以s为单位
	 * 更新处理噪声的协方差矩阵
	 */
	double delta_t = (double(meas_package.timestamp_) - double(previous_timestamp_)) / 1000000.0;
	float delta_t2 = delta_t*delta_t;
	float delta_t3 = delta_t2*delta_t;
	float delta_t4 = delta_t3*delta_t;
	//更新F_矩阵
	ekf_.F_(0, 2) = delta_t;
	ekf_.F_(1, 3) = delta_t;
	//更新Q_矩阵
	ekf_.Q_ = Eigen::MatrixXd(4,4);
	float noise_ax2 = 9.0;
	float noise_ay2 = 9.0;
	ekf_.Q_ << delta_t4 / 4 * noise_ax2, 0, delta_t3 / 2 * noise_ax2, 0,
				0,   delta_t4 / 4 * noise_ay2, 0, delta_t3 / 2 * noise_ay2,
			   delta_t3 / 2 * noise_ax2, 0, delta_t2*noise_ax2, 0,
		        0,   delta_t3 / 2 * noise_ay2, 0, delta_t2*noise_ay2;
	//进行预测
	ekf_.Predict();
	/*
	 * 更新
	 * 基于传感器的类型选择更新的步骤
	 */
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		//radar的更新
		ekf_.H_ = CalculateJacobian_ctrv(ekf_.x_);//状态空间到测量空间的映射矩阵,计算雅克比矩阵
		ekf_.R_ = R_radar_;//传入测量误差
        ekf_.UpdateEKF(meas_package.raw_measurements_);//对于radar数据采用扩展卡尔曼滤波更新
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
		//lidar的更新
		ekf_.H_ = Eigen::MatrixXd(2,4);//状态空间到测量空间的映射矩阵
		ekf_.H_ << 1, 0, 0, 0,
			0, 1, 0, 0;
		ekf_.R_ = R_laser_;//传入测量误差
		ekf_.Update(meas_package.raw_measurements_);//对于lidar数据采用线性卡尔曼滤波更新
    }
	/*
	 * 完成更新，更新时间
	 */
	previous_timestamp_ = meas_package.timestamp_;
}