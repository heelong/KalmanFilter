#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"
const double DoublePI = 2 * M_PI;
class MeasurementPackage {
public:
  double timestamp_;

  enum SensorType{
    LASER,
	LASER_RADAR,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;
  MeasurementPackage(){};
};

#endif /* MEASUREMENT_PACKAGE_H_ */
