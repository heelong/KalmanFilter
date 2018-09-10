#include "kf_Fusion.h"
#include <iostream>
#include <time.h>

/**
 * Initializes  Kalman filter
 */
KF_FUSION::KF_FUSION() {
	is_initialized_ = false;
	previous_timestamp_ = 0;


	H_laser_ = Eigen::MatrixXd(2,4);
	H_laser_ << 1, 0, 0, 0, 
		0, 1, 0, 0;
	H_radar_ = Eigen::MatrixXd(4, 4);
	H_radar_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	H_laser_radar_ = Eigen::MatrixXd(4, 4);
	H_laser_radar_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;


	float std_laspx_ = 0.05;
	float std_laspy_ = 0.05;
	R_laser_ = Eigen::MatrixXd(2, 2);
    R_laser_ << std_laspx_*std_laspx_, 0,
            0, std_laspy_*std_laspy_;

	float std_radpx_ = 0.3;
	float std_radpy_ = 0.3;
	float std_vx_ = 1.3;
	float std_vy_ = 1.3;
	R_radar_ = Eigen::MatrixXd(4, 4);
	R_radar_ << std_radpx_*std_radpx_, 0, 0,0,
		0, std_radpy_*std_radpy_,0, 0,
		0, 0, std_vx_*std_vx_,0,
		0, 0, 0, std_vy_*std_vy_;

	R_laser_radar_ = Eigen::MatrixXd(4, 4);
	R_laser_radar_ << std_laspx_*std_laspx_, 0, 0, 0,
		0, std_laspy_*std_laspy_, 0, 0,
		0, 0, std_vx_*std_vx_, 0,
		0, 0, 0, std_vx_*std_vx_;

	//状态向量
	ekf_.x_ = Eigen::VectorXd(4);
	//系统状态不确定性
	ekf_.P_ = Eigen::MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 0.5, 0,
		0, 0, 0, 0.5;
	ekf_.F_ = Eigen::MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;
	ekf_.Q_ = Eigen::MatrixXd(4, 4);
}

KF_FUSION::~KF_FUSION() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void KF_FUSION::ProcessMeasurement(const MeasurementPackage &meas_package) {
	clock_t start, finish;
	start = clock();
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
			ekf_.x_[0] = meas_package.raw_measurements_[0];
			ekf_.x_[1] = meas_package.raw_measurements_[1];
			ekf_.x_[2] = 0;
			ekf_.x_[3] = 0;
		}
		else {
			ekf_.x_[0] = meas_package.raw_measurements_[0];
			ekf_.x_[1] = meas_package.raw_measurements_[1];
			ekf_.x_[2] = meas_package.raw_measurements_[2];
			ekf_.x_[3] = meas_package.raw_measurements_[3];
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
		ekf_.H_ = H_radar_;
		ekf_.R_ = R_radar_;//传入测量误差
		ekf_.Update(meas_package.raw_measurements_);//对于radar数据采用扩展卡尔曼滤波更新
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
		//lidar的更新
		ekf_.H_ = H_laser_;
		ekf_.R_ = R_laser_;//传入测量误差
		ekf_.Update(meas_package.raw_measurements_);//对于lidar数据采用线性卡尔曼滤波更新
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER_RADAR) {
		//lidar的更新
		ekf_.H_ = H_laser_radar_;
		ekf_.R_ = R_laser_radar_;//传入测量误差
		ekf_.Update(meas_package.raw_measurements_);//近似默认为线性模型
	}
	/*
	 * 完成更新，更新时间
	 */
	previous_timestamp_ = meas_package.timestamp_;
	finish = clock();
	float totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	//std::cout << " -------------Current Frame cluster Time:" << totaltime*1000.0 << "ms" << std::endl;
}

void KF_FUSION::getState(Eigen::VectorXd& x)
{
	x[0] = ekf_.x_[0];
	x[1] = ekf_.x_[1];
	x[2] = ekf_.x_[2];
	x[3] = ekf_.x_[3];
}
