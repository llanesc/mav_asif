#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include "mav_control_router.hpp"
#include <chrono>
#include <iostream>
#include "Eigen/Dense"
#include "mav_util.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace asif;

class Mav1ASIF: public rclcpp::Node {
public:
	Mav1ASIF()
			: Node("mav1_node"), mav_control_router_(0)
	{

		mav1_estimator_odometry_sub_ =
				this->create_subscription<VehicleOdometry>(mav1_namespace_ + "fmu/vehicle_odometry/out", 10,
				                                           [this](const VehicleOdometry::UniquePtr msg)
				                                           {
					                                           mav1_odom_ = *msg;
				                                           });
		mav2_estimator_odometry_sub_ =
				this->create_subscription<VehicleOdometry>(mav2_namespace_ + "fmu/vehicle_odometry/out", 10,
				                                           [this](const VehicleOdometry::UniquePtr msg)
				                                           {
					                                           mav2_odom_ = *msg;
				                                           });


		mav_channels_sub_ =
				this->create_subscription<RcChannels>(mav1_namespace_ + "fmu/rc_channels/out", 10,
				                                      [this](const RcChannels::UniquePtr msg)
				                                      {
					                                      mav_channels_ = *msg;
					                                      if (mav_channels_.channels[POSITION_SETPOINT_CHANNEL - 1] >= 0.75){
						                                      mav1_des_ = {0.0, -0.5, -2.0, 0.0}; //x, y, z, yaw
						                                      mav2_des_ = {0.0, 0.0, -1.5, 0.0}; //x, y, z, yaw
					                                      } else {
						                                      mav1_des_ = {0.0, 0.0, -1.5, 0.0}; //x, y, z, yaw
						                                      mav2_des_ = {0.0, 0.5, -1.0, 0.0}; //x, y, z, yaw
					                                      }
				                                      });

		mav1_des_ = {0.0, 0.0, -1.5, 0.0}; //x, y, z, yaw
		mav2_des_ = {0.0, 0.5, -1.0, 0.0}; //x, y, z, yaw

		auto timer_callback = [this]() -> void
		{
			mav_control_router_.position_controller(mav1_odom_,
			                                        mav2_odom_,
			                                        mav1_control_,
			                                        mav2_control_,
			                                        mav1_des_,
			                                        mav2_des_);
			mav_control_router_.set_control(mav1_odom_,
			                                mav2_odom_,
			                                mav1_control_,
			                                mav2_control_);
		};

		timer_ = this->create_wall_timer(10ms, timer_callback);

	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr mav_channels_sub_;
	rclcpp::Subscription<VehicleOdometry>::SharedPtr mav1_estimator_odometry_sub_;
	rclcpp::Subscription<VehicleOdometry>::SharedPtr mav2_estimator_odometry_sub_;

	MavControlRouter mav_control_router_;
	VehicleOdometry mav1_odom_;
	VehicleOdometry mav2_odom_;
	mavControl mav1_control_;
	mavControl mav2_control_;
	std::array<double, 4> mav1_des_;
	std::array<double, 4> mav2_des_;
	RcChannels mav_channels_;

	std::string mav1_namespace_ = "/mav1/";
	std::string mav2_namespace_ = "/mav2/";

};


int main(int argc, char *argv[])
{
	std::cout << "Starting MAV1 ASIF example..." << std::endl;
	Eigen::initParallel();
	Eigen::setNbThreads(16);
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Mav1ASIF>());

	rclcpp::shutdown();
	return 0;
}

