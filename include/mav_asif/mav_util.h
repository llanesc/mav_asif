#include "Eigen/Core"



static Eigen::Vector3d quaternion_to_rpy_wrap(const Eigen::Quaterniond &q)
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