#ifndef EKF_EKF_H
#define EKF_EKF_H


#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kf.h"

class EKF {
public:
    /**
     * Constructor
     */
    EKF();

    /**
     * Destructor
     */
    virtual ~EKF();

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(const MeasurementPackage &meas_package);

	//卡尔曼滤波器对象
	KF ekf_;

private:
	//判断是否被初始化
	bool is_initialized_;

	// 上一数据时间戳
	long long previous_timestamp_;
	
	Eigen::MatrixXd R_laser_;//激光雷达的误差矩阵
	Eigen::MatrixXd R_radar_;//毫米波雷达的误差矩阵
	Eigen::MatrixXd H_laser_;//激光雷达映射矩阵
	Eigen::MatrixXd Hj_;//毫米波雷达映射矩阵对应的雅克比矩阵
};


#endif //EKF_EKF_H
