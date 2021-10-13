#include "mav_control_router.hpp"
#include <utility>
#include "workspace.h"
#include "Eigen/Dense"
#include <px4_ros_com/frame_transforms.h>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_ros_com::frame_transforms::utils::quaternion;
using namespace px4_ros_com::frame_transforms;
using namespace mav_asif_msgs::msg;
using namespace px4_msgs::msg;
using namespace asif;

MavControlRouter::MavControlRouter(uint8_t mav_id)
        : Node("mav_control_router"), mav_id_(mav_id) {
    // ----------------------- Parameter Initialization ------------------
    declare_parameter("mass1");
    declare_parameter("mass2");
    declare_parameter("gravity");
    declare_parameter("safe_displacement.y");
    declare_parameter("safe_displacement.z");
    declare_parameter("backup_dynamics.spring_constantX");
    declare_parameter("backup_dynamics.spring_constantYZ");
    declare_parameter("backup_dynamics.spring_dampenerX");
    declare_parameter("backup_dynamics.spring_dampenerYZ");
    declare_parameter("backup_dynamics.spring_saturation");
    declare_parameter("backup_dynamics.roll_kp");
    declare_parameter("backup_dynamics.pitch_yaw_kp");
    declare_parameter("thrust_model.mav_max_thrust");
    declare_parameter("thrust_model.mav_min_thrust");
    try {
        get_parameter("mass1", mass1_);
        get_parameter("mass2", mass2_);
        get_parameter("gravity", gravity_);
        get_parameter("backup_dynamics.spring_constantX", spring_constantX_);
        get_parameter("backup_dynamics.spring_constantYZ", spring_constantYZ_);
        get_parameter("backup_dynamics.spring_dampenerX", spring_dampenerX_);
        get_parameter("backup_dynamics.spring_dampenerYZ", spring_dampenerYZ_);
        get_parameter("backup_dynamics.spring_saturation", spring_saturation_);
        get_parameter("backup_dynamics.roll_kp", roll_kp_);
        get_parameter("backup_dynamics.pitch_yaw_kp", pitch_yaw_kp_);
        get_parameter("thrust_model.mav_max_thrust", mav_max_thrust);
        get_parameter("thrust_model.mav_min_thrust", mav_min_thrust);
    } catch (rclcpp::ParameterTypeException &excp) {
        RCLCPP_ERROR(get_logger(), "Parameter type exception caught");
        rclcpp::shutdown(nullptr, "Parameter type exception caught on initialization");
    }
    // ----------------------- Publishers --------------------------
    vehicle_command_pub_ =
            this->create_publisher<px4_msgs::msg::VehicleCommand>(
                    "mav" + std::to_string(mav_id_) + "/fmu/vehicle_command/in", 10);
    offboard_control_mode_pub_ =
            this->create_publisher<px4_msgs::msg::OffboardControlMode>(
                    "mav" + std::to_string(mav_id_) + "/fmu/offboard_control_mode/in", 10);
    asif_status_pub_ =
            this->create_publisher<AsifStatus>("mav" + std::to_string(mav_id_) + "/asif_status", 10);
    vehicle_rates_setpoint_pub_ =
            this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
                    "mav" + std::to_string(mav_id_) + "/fmu/vehicle_rates_setpoint/in", 10);
    // ----------------------- Subscribers --------------------------
    mav_battery_status_sub_ =
            this->create_subscription<px4_msgs::msg::BatteryStatus>(
                    "mav" + std::to_string(mav_id_) + "/fmu/battery_status/out", 10,
                    [this](const BatteryStatus::UniquePtr msg) {
                        mav_battery_status_ = *msg;
                    });
    timesync_sub_ =
            this->create_subscription<px4_msgs::msg::Timesync>("mav" + std::to_string(mav_id_) + "/fmu/timesync/out",
                                                               10,
                                                               [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
                                                                   timestamp_.store(msg->timestamp);
                                                               });
    mav_vehicle_status_sub_ =
            this->create_subscription<px4_msgs::msg::VehicleStatus>(
                    "mav" + std::to_string(mav_id_) + "/fmu/vehicle_status/out", 10,
                    [this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
                        mav_vehicle_status_ = *msg;
                    });
    mav_channels_sub_ =
            this->create_subscription<px4_msgs::msg::RcChannels>(
                    "mav" + std::to_string(mav_id_) + "/fmu/rc_channels/out", 10,
                    [this](const px4_msgs::msg::RcChannels::UniquePtr msg) {
                        mav_channels_ = *msg;
                        if (msg->channels[ASIF_ENABLE_CHANNEL - 1] >= 0.75) {
                            asif_enabled_ = true;
                        } else {
                            asif_enabled_ = false;
                        }

                        if (mav_channels_.channels[POSITION_SETPOINT_CHANNEL - 1] >= 0.75) {
                            mav1_des_ = {0.0, -1.0, -3.0, 0.0}; //x, y, z, yaw
                            mav2_des_ = {0.0, 0.0, -2.0, 0.0}; //x, y, z, yaw
                        } else {
                            mav1_des_ = {0.0, 0.0, -2.0, 0.0}; //x, y, z, yaw
                            mav2_des_ = {0.0, 1.0, -1.0, 0.0}; //x, y, z, yaw
                        }
                    });
    mav1_estimator_odometry_sub_ =
            this->create_subscription<VehicleOdometry>("mav" + std::to_string(mav_id_) + "/fmu/vehicle_odometry/out", 10,
                                                       [this](const VehicleOdometry::UniquePtr msg) {
                                                           mav1_odom_ = *msg;
                                                       });
    mav2_estimator_odometry_sub_ =
            this->create_subscription<VehicleOdometry>("mav" + std::to_string(mav_id_) + "/fmu/vehicle_odometry/out", 10,
                                                       [this](const VehicleOdometry::UniquePtr msg) {
                                                           mav2_odom_ = *msg;
                                                       });

    auto timer_callback = [this]() -> void
    {
        position_controller();
        set_control();
    };

    timer_ = create_wall_timer(10ms, timer_callback);
}

/**
* @brief Publish the offboard control mode.
*        For this example, only position and altitude controls are active.
*/
void MavControlRouter::publish_offboard_control_mode() const {
    OffboardControlMode msg{};
    msg.timestamp = timestamp_.load();
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = true;

    offboard_control_mode_pub_->publish(msg);
}


/**
 * @brief Send a command to Arm the vehicle
 */
void MavControlRouter::arm() const {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void MavControlRouter::disarm() const {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void MavControlRouter::publish_vehicle_command(uint16_t command, float param1,
                                               float param2) const {
    VehicleCommand msg{};
    msg.timestamp = timestamp_.load();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_pub_->publish(msg);
}

void MavControlRouter::set_control() {
    mav_asif_msgs::msg::AsifStatus asif_status;
    asif_status.position_controller[0].roll_rate = mav1_control_.roll_rate;
    asif_status.position_controller[0].pitch_rate = mav1_control_.pitch_rate;
    asif_status.position_controller[0].yaw_rate = mav1_control_.yaw_rate;
    asif_status.position_controller[0].thrust = mav1_control_.thrust;
    asif_status.position_controller[1].roll_rate = mav2_control_.roll_rate;
    asif_status.position_controller[1].pitch_rate = mav2_control_.pitch_rate;
    asif_status.position_controller[1].yaw_rate = mav2_control_.yaw_rate;
    asif_status.position_controller[1].thrust = mav2_control_.thrust;

    if (asif_enabled_) {
        asif_solver_.QP(&workspace,
                        mav1_odom_,
                        mav2_odom_,
                        mav1_control_,
                        mav2_control_,
                        asif_status.worst_barrier,
                        asif_status.worst_corner_time);
        asif_status.active_set_invariance_filter[0].roll_rate = mav1_control_.roll_rate;
        asif_status.active_set_invariance_filter[0].pitch_rate = mav1_control_.pitch_rate;
        asif_status.active_set_invariance_filter[0].yaw_rate = mav1_control_.yaw_rate;
        asif_status.active_set_invariance_filter[0].thrust = mav1_control_.thrust;
        asif_status.active_set_invariance_filter[1].roll_rate = mav2_control_.roll_rate;
        asif_status.active_set_invariance_filter[1].pitch_rate = mav2_control_.pitch_rate;
        asif_status.active_set_invariance_filter[1].yaw_rate = mav2_control_.yaw_rate;
        asif_status.active_set_invariance_filter[1].thrust = mav2_control_.thrust;
    }
    asif_status.asif_active = asif_enabled_;
    asif_status.timestamp = this->now().nanoseconds();
    asif_status_pub_->publish(asif_status);

    publish_control();
}

void MavControlRouter::publish_control() {
    VehicleRatesSetpoint vehicle_rates;
    if ((mav_channels_.channels[OFFBOARD_ENABLE_CHANNEL - 1] >= 0.75) &&
        (mav_vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)) {
        if ((mav_vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) &&
            (offboard_counter_ == 10)) {
            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        } else if (offboard_counter_ < 11) {
            offboard_counter_++;
        }
        switch (mav_id_) {
            case 0:
                vehicle_rates.timestamp = timestamp_.load();
                vehicle_rates.roll = mav1_control_.roll_rate;
                vehicle_rates.pitch = mav1_control_.pitch_rate;
                vehicle_rates.yaw = mav1_control_.yaw_rate;
                vehicle_rates.thrust_body[2] = -compute_relative_thrust(mav1_control_.thrust);
                break;
            case 1:
                vehicle_rates.timestamp = timestamp_.load();
                vehicle_rates.roll = mav2_control_.roll_rate;
                vehicle_rates.pitch = mav2_control_.pitch_rate;
                vehicle_rates.yaw = mav2_control_.yaw_rate;
                vehicle_rates.thrust_body[2] = -compute_relative_thrust(mav2_control_.thrust);
                break;
            default:
                vehicle_rates.timestamp = timestamp_.load();
                vehicle_rates.roll = 0;
                vehicle_rates.pitch = 0;
                vehicle_rates.yaw = 0;
                vehicle_rates.thrust_body[2] = 0;
        }
        publish_offboard_control_mode();
        vehicle_rates_setpoint_pub_->publish(vehicle_rates);
    } else {
        offboard_counter_ = 0;
    }
}

void MavControlRouter::position_controller() {
    double f_cmd_x1 = spring_constantX_ * tanh(spring_saturation_ * (mav1_des_[0] - mav1_odom_.x)) -
                      spring_dampenerX_ * mav1_odom_.vx;
    double f_cmd_x2 = spring_constantX_ * tanh(spring_saturation_ * (mav2_des_[0] - mav2_odom_.x)) -
                      spring_dampenerX_ * mav2_odom_.vx;
    double f_cmd_y1 = spring_constantYZ_ * tanh(spring_saturation_ * (mav1_des_[1] - mav1_odom_.y)) -
                      spring_dampenerYZ_ * mav1_odom_.vy;
    double f_cmd_y2 = spring_constantYZ_ * tanh(spring_saturation_ * (mav2_des_[1] - mav2_odom_.y)) -
                      spring_dampenerYZ_ * mav2_odom_.vy;
    double f_cmd_z1 = spring_constantYZ_ * tanh(spring_saturation_ * (mav1_des_[2] - mav1_odom_.z)) - mass1_ * gravity_ -
                      spring_dampenerYZ_ * mav1_odom_.vz;
    double f_cmd_z2 = spring_constantYZ_ * tanh(spring_saturation_ * (mav2_des_[2] - mav2_odom_.z)) - mass2_ * gravity_ -
                      spring_dampenerYZ_ * mav2_odom_.vz;

    Eigen::Vector3d thrust_dir1{f_cmd_x1, f_cmd_y1, f_cmd_z1};
    Eigen::Vector3d thrust_dir2{f_cmd_x2, f_cmd_y2, f_cmd_z2};
    thrust_dir1.normalize();
    thrust_dir2.normalize();

    Eigen::Vector3d eulers1 = quaternion_to_rpy_wrap(Eigen::Quaterniond(mav1_odom_.q[0],
                                                                        mav1_odom_.q[1],
                                                                        mav1_odom_.q[2],
                                                                        mav1_odom_.q[3]));
    Eigen::Vector3d eulers2 = quaternion_to_rpy_wrap(Eigen::Quaterniond(mav2_odom_.q[0],
                                                                        mav2_odom_.q[1],
                                                                        mav2_odom_.q[2],
                                                                        mav2_odom_.q[3]));

    Eigen::Vector3d iterm_thrust_frame1_1 = ned_to_aircraft_frame(thrust_dir1, quaternion_from_euler(0, 0, eulers1(2)));
    Eigen::Vector3d iterm_thrust_frame1_2 = ned_to_aircraft_frame(thrust_dir2, quaternion_from_euler(0, 0, eulers2(2)));

    double theta1_cmd = atan2(-iterm_thrust_frame1_1.x(), -iterm_thrust_frame1_1.z());
    double theta2_cmd = atan2(-iterm_thrust_frame1_2.x(), -iterm_thrust_frame1_2.z());

    Eigen::Vector3d iterm_thrust_frame2_1 = ned_to_aircraft_frame<Eigen::Vector3d>(iterm_thrust_frame1_1,
                                                                                   quaternion_from_euler(0,
                                                                                                         theta1_cmd,
                                                                                                         0));
    Eigen::Vector3d iterm_thrust_frame2_2 = ned_to_aircraft_frame<Eigen::Vector3d>(iterm_thrust_frame1_2,
                                                                                   quaternion_from_euler(0,
                                                                                                         theta2_cmd,
                                                                                                         0));
    double phi1_cmd = atan2(iterm_thrust_frame2_1.y(), -iterm_thrust_frame2_1.z());
    double phi2_cmd = atan2(iterm_thrust_frame2_2.y(), -iterm_thrust_frame2_2.z());

    mav1_control_.thrust = -f_cmd_x1 * cos(eulers1.x()) * sin(eulers1.y()) + f_cmd_y1 * sin(eulers1.x())
                          - f_cmd_z1 * cos(eulers1.x()) * cos(eulers1.y());
    mav2_control_.thrust = -f_cmd_x2 * cos(eulers2.x()) * sin(eulers2.y()) + f_cmd_y2 * sin(eulers2.x())
                          - f_cmd_z2 * cos(eulers2.x()) * cos(eulers2.y());

    mav1_control_.roll_rate = -roll_kp_ * (eulers1.x() - phi1_cmd);
    mav2_control_.roll_rate = -roll_kp_ * (eulers2.x() - phi2_cmd);

    mav1_control_.pitch_rate = -pitch_yaw_kp_ * (eulers1.y() - theta1_cmd);
    mav2_control_.pitch_rate = -pitch_yaw_kp_ * (eulers2.y() - theta2_cmd);

    mav1_control_.yaw_rate = -pitch_yaw_kp_ * (eulers1.z() - mav1_des_[3]);
    mav2_control_.yaw_rate = -pitch_yaw_kp_ * (eulers2.z() - mav2_des_[3]);
}

double MavControlRouter::compute_relative_thrust(const double &collective_thrust) const {
    if (mav_battery_status_.voltage_filtered_v > 14.0) {
        double rel_thrust = (collective_thrust - mav_min_thrust) / (mav_max_thrust - mav_min_thrust);
        return (0.54358075 * rel_thrust + 0.25020242 * sqrt(3.6484 * rel_thrust + 0.00772641) - 0.021992793) *
               (1 - 0.0779 * (mav_battery_status_.voltage_filtered_v - 16.0));
    } else {
        double rel_thrust = (collective_thrust - mav_min_thrust) / (mav_max_thrust - mav_min_thrust);
        return (0.54358075 * rel_thrust + 0.25020242 * sqrt(3.6484 * rel_thrust + 0.00772641) - 0.021992793);
    }
}
