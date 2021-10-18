#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/rc_channels.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <mav_asif_msgs/msg/asif_status.hpp>
#include "mav_util.h"
#include "mav_asif_util.hpp"

namespace asif
{

class MavControlRouter: public rclcpp::Node {
public:
	MavControlRouter(uint8_t mav_id);

private:
	// ----------------------- Publishers --------------------------
	rclcpp::Publisher<mav_asif_msgs::msg::AsifStatus>::SharedPtr asif_status_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
	// ----------------------- Subscribers --------------------------
	rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr mav_channels_sub_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr mav_battery_status_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr mav_vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr mav1_estimator_odometry_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr mav2_estimator_odometry_sub_;

	// Message Callback Variables
	std::atomic<uint64_t> timestamp_;
	px4_msgs::msg::BatteryStatus mav_battery_status_;
	px4_msgs::msg::VehicleStatus mav_vehicle_status_;
	px4_msgs::msg::RcChannels mav_channels_;
    px4_msgs::msg::VehicleOdometry mav1_odom_;
    px4_msgs::msg::VehicleOdometry mav2_odom_;

	// Class Variables
    rclcpp::TimerBase::SharedPtr timer_;
	ASIF asif_solver_;
	uint8_t mav_id_;
	mavControl mav1_control_;
	mavControl mav2_control_;
	bool asif_enabled_{false};
	double gravity_;
	double mass1_;
	double mass2_;
	double spring_constantX_;
	double spring_constantYZ_;
	double spring_dampenerX_;
	double spring_dampenerYZ_;
	double spring_saturation_;
	double roll_kp_;
	double pitch_yaw_kp_;
	double mav_max_thrust;
	double mav_min_thrust;
#ifdef RUN_SITL
	rclcpp::TimerBase::SharedPtr asif_activate_timer_;
#endif

    std::array<double, 4> mav1_des_ = {0.0, 0.0, -2.0, 0.0}; //x, y, z, yaw
    std::array<double, 4> mav2_des_ = {0.0, 1.0, -1.0, 0.0}; //x, y, z, yaw
	uint8_t offboard_counter_{0};

	// Class methods
	void arm() const;
	void disarm() const;
	void publish_offboard_control_mode() const;
	void publish_control();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
	                             float param2 = 0.0) const;

	double compute_relative_thrust(const double &collective_thrust) const;
    void set_control();
    void position_controller();
}; //class MavControlRouter
} //namespace asif
