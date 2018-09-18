#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"
const double DoublePI = 2 * M_PI;
//目标属性来源是多元化的,不同方位、距离、高度下，目标不会被所有传感器探测到,本枚举标明障碍物融合所用到的传感器信息
enum ScanningAttributes  //属性来源
{
	SA_Lidar = 1,	//只使用Lidar探测到目标的属性
	SA_Radar = 2,	//只使用Radar探测到目标的属性
	SA_Camera = 3,	//只使用Camera探测到目标的属性
	SA_Lidar_Radar = 4,		//Lidar+Radar融合
	SA_Lidar_Camera = 5,	//Lidar+Camera融合
	SA_Radar_Camera = 6,	//Radar+Camera融合
	SA_Lidar_Radar_Camera = 7,	//Lidar+Radar+Camera融合
};

//障碍物（目标）信息
typedef struct Obj
{
	//1-目标基本信息///////////////////////////////////
	short index;                //目标跟踪信息，几号目标
	ScanningAttributes ObstacleAttriSrc;  //障碍物属性传感器识别来源

	float Direction_Local;	//包络盒的航向角,局部坐标系下（目标的运动方向，正前为0，顺时针增加 0-360°）
	float Direction_Global;	//包络盒的航向角,大地坐标系下（目标的运动方向，正北为0，正东90°）

	//2-局部坐标系（车辆坐标系）目标信息////////////////  所有坐标位置为右手系（正右为X，正前为Y，正上为Z），单位为m
	double  CenterPoint_Local_x;  //中心点坐标
	double  CenterPoint_Local_y;  //中心点坐标
	double ObjSpeed_Local_x;
	double ObjSpeed_Local_y;
	float	Turn_Rate;	//目标转率
	float	Acceleration_Local;	//加速度（沿航向）

	//毫米波测量信息
	double Range;
	double RelativeVelocity;
	double RelativeAngle;

	//3-目标时间信息/////////////////////////////
	int	Type_Probs;  //分类可靠性（置信度）
	double CurrentTime;	//当前帧被扫描时的时间（或者最后一次出现的时间）--UTC时间

}Obj;

class MeasurementPackage {
public:
  long timestamp_;

  enum SensorType{
    LASER,
	LASER_RADAR,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;
  MeasurementPackage(){};
  //MeasurementPackage(Obj &Obstacles)
  //{
	 // if (Obstacles.ObstacleAttriSrc == SA_Lidar) {
		//  // LASER MEASUREMENT

		//  // read measurements at this timestamp
		//  sensor_type_ = MeasurementPackage::LASER;
		//  raw_measurements_ = Eigen::VectorXd(2);
		//  raw_measurements_ << Obstacles.CenterPoint_Local_x, Obstacles.CenterPoint_Local_y;
		//  timestamp_ = Obstacles.CurrentTime/**100000*/;
	 // }
	 // else if (Obstacles.ObstacleAttriSrc == SA_Radar) {
		//  // RADAR MEASUREMENT
		//  sensor_type_ = MeasurementPackage::RADAR;
		//  raw_measurements_ = Eigen::VectorXd(3);
		//  raw_measurements_ << Obstacles.Range, Obstacles.RelativeAngle, Obstacles.RelativeVelocity;
		//  timestamp_ = Obstacles.CurrentTime /** 100000*/;
	 // }
  //}
};

#endif /* MEASUREMENT_PACKAGE_H_ */
