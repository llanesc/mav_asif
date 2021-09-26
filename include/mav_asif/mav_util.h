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

static double compute_relative_thrust(const double &collective_thrust, const double &voltage, const double &min_thrust, const double &max_thrust)
{
    static constexpr double k1 = -0.003594410861473;
    static constexpr double k2 = 0.090929073038638;
    static constexpr double k3 = -0.395016026779458;
    static constexpr double hover_thrust = 0.1626; // hover thrust at 14.85V
    if (voltage > 14.0) {
        return (k1 * voltage * voltage + k2 * voltage + k3) / hover_thrust * (collective_thrust - min_thrust) / (max_thrust - min_thrust);
    } else {
        return (collective_thrust - min_thrust) / (max_thrust - min_thrust);
    }

}