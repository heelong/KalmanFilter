#ifndef KF_KF_FUSION_H
#define KF_KF_FUSION_H


#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kf.h"



//只考虑线性化的结果
class KF_FUSION {
	//输入的数据为激光雷达数据、毫米波数据（转笛卡尔坐标系）以及激光+毫米波雷达（转笛卡尔坐标系）数据
public:
    /**
     * Constructor
     */
	KF_FUSION();

    /**
     * Destructor
     */
	virtual ~KF_FUSION();

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(const MeasurementPackage &meas_package);

	//卡尔曼滤波器对象
	KF ekf_;
	void getState(Eigen::VectorXd& x);
	void initial();
private:
	//判断是否被初始化
	bool is_initialized_;

	// 上一数据时间戳
	long long previous_timestamp_;
	
	Eigen::MatrixXd R_laser_;//激光雷达的误差矩阵
	Eigen::MatrixXd R_radar_;//毫米波雷达的误差矩阵
	Eigen::MatrixXd R_laser_radar_;//毫米波雷达的误差矩阵
	Eigen::MatrixXd H_laser_;//激光雷达映射矩阵
	Eigen::MatrixXd H_radar_;//毫米波雷达映射矩阵
	Eigen::MatrixXd H_laser_radar_;//毫米波雷达映射矩阵
};


#endif //EKF_EKF_H
