#pragma once

#include "Eigen/Dense"

namespace asif
{

static constexpr uint8_t OFFBOARD_ENABLE_CHANNEL = 8;
static constexpr uint8_t POSITION_SETPOINT_CHANNEL = 9;
static constexpr uint8_t ASIF_ENABLE_CHANNEL = 10;

typedef struct mavControl {
	double roll_rate; // rad/s
	double pitch_rate; // rad/s
	double yaw_rate; // rad/s
	double thrust; // Newtons
#if defined(__cplusplus)
	mavControl()
			: roll_rate(0)
			, pitch_rate(0)
			, yaw_rate(0)
			, thrust(0) {};
#endif
} mavControl;


Eigen::Vector3d quaternion_to_rpy_wrap(const Eigen::Quaterniond &q)
{
	Eigen::Vector3d rpy;
	double roll = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (pow(q.x(), 2) + pow(q.y(), 2)));
	double pitch = asin(2 * (q.w() * q.y() - q.z() * q.x()));
	double yaw = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (pow(q.y(), 2) + pow(q.z(), 2)));

	rpy << roll,
			pitch,
			yaw;

	return rpy;
}


}