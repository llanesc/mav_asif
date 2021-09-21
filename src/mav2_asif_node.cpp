#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_ros_com/frame_transforms.h>
#include <rclcpp/rclcpp.hpp>
#include "workspace.h"
#include "mav_asif_util.h"
#include <chrono>
#include <iostream>
#include "Eigen/Core"
#include "mav_util.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace asif;
using namespace px4_ros_com::frame_transforms::utils::quaternion;
using namespace px4_ros_com::frame_transforms;

class Mav2ASIF : public rclcpp::Node {
public:
    Mav2ASIF()
            : Node("mav2_node") {
        this->declare_parameter("mass1");
        this->declare_parameter("mass2");
        this->declare_parameter("gravity");
        this->declare_parameter("safe_displacement.y");
        this->declare_parameter("safe_displacement.z");
        this->declare_parameter("backup_dynamics.spring_constantX");
        this->declare_parameter("backup_dynamics.spring_constantYZ");
        this->declare_parameter("backup_dynamics.spring_dampenerX");
        this->declare_parameter("backup_dynamics.spring_dampenerYZ");
        this->declare_parameter("backup_dynamics.spring_saturation");
        this->declare_parameter("backup_dynamics.roll_kp");
        this->declare_parameter("backup_dynamics.pitch_yaw_kp");
        this->declare_parameter("quad1_max_thrust_body");
        this->declare_parameter("quad2_max_thrust_body");
        this->declare_parameter("quad1_min_thrust_body");
        this->declare_parameter("quad2_min_thrust_body");

        try {
            this->get_parameter("mass1", mass1_);
            this->get_parameter("mass2", mass2_);
            this->get_parameter("gravity", gravity_);
            this->get_parameter("safe_displacement.y", safe_displacement_y_);
            this->get_parameter("safe_displacement.z", safe_displacement_z_);
            this->get_parameter("backup_dynamics.spring_constantX", spring_constantX_);
            this->get_parameter("backup_dynamics.spring_constantYZ", spring_constantYZ_);
            this->get_parameter("backup_dynamics.spring_dampenerX", spring_dampenerX_);
            this->get_parameter("backup_dynamics.spring_dampenerYZ", spring_dampenerYZ_);
            this->get_parameter("backup_dynamics.spring_saturation", spring_saturation_);
            this->get_parameter("backup_dynamics.roll_kp", roll_kp_);
            this->get_parameter("backup_dynamics.pitch_yaw_kp", pitch_yaw_kp_);
            this->get_parameter("quad1_max_thrust_body", quad1_max_thrust_body_);
            this->get_parameter("quad2_max_thrust_body", quad2_max_thrust_body_);
            this->get_parameter("quad1_min_thrust_body", quad1_min_thrust_body_);
            this->get_parameter("quad2_min_thrust_body", quad2_min_thrust_body_);
        } catch (rclcpp::ParameterTypeException &excp) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Parameter type exception caught");
            rclcpp::shutdown(nullptr, "ASIF ERROR: Parameter type exception caught on initialization");
        }
        offboard_control_mode_publisher_ =
                this->create_publisher<OffboardControlMode>(mav2_namespace_ + "fmu/offboard_control_mode/in", 10);
        vehicle_rates_setpoint_publisher_ =
                this->create_publisher<VehicleRatesSetpoint>(mav2_namespace_ + "fmu/vehicle_rates_setpoint/in", 10);
        vehicle_command_publisher_ =
                this->create_publisher<VehicleCommand>(mav2_namespace_ + "fmu/vehicle_command/in", 10);

        // get common timestamp
        timesync_sub_ =
                this->create_subscription<px4_msgs::msg::Timesync>(mav2_namespace_ + "fmu/timesync/out", 10,
                                                                   [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
                                                                       timestamp_.store(msg->timestamp);
                                                                   });
        mav1_estimator_odometry_subscriber_ =
                this->create_subscription<VehicleOdometry>(mav1_namespace_ + "fmu/vehicle_odometry/out", 10,
                                                           [this](const VehicleOdometry::UniquePtr msg) {
                                                               mav1_ = *msg;
                                                           });
        mav2_estimator_odometry_subscriber_ =
                this->create_subscription<VehicleOdometry>(mav2_namespace_ + "fmu/vehicle_odometry/out", 10,
                                                           [this](const VehicleOdometry::UniquePtr msg) {
                                                               mav2_ = *msg;
                                                           });
        offboard_setpoint_counter_ = 0;

        mav1_des_ << 0.0, 0.0, -2.0, 0.0; //x, y, z, yaw
        mav2_des_ << 0.0, 1.0, -1.0, 0.0; //x, y, z, yaw

        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                // Change to Offboard mode after 10 setpoints
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

                // Arm the vehicle
                this->arm();
            }

            // offboard_control_mode needs to be paired with trajectory_setpoint
            publish_offboard_control_mode();
            if (asif_active_) {
                geometric_controller(mav1_des_, mav2_des_, control1_, control2_);
                asif_solver_.QP(&workspace, mav1_, mav2_, &control1_, &control2_);
            } else {
                geometric_controller(mav1_des_, mav2_des_, control1_, control2_);
            }

            control2_.timestamp = timestamp_.load();
            vehicle_rates_setpoint_publisher_->publish(control2_);

            // stop the counter after reaching 11
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        };
        timer_ = this->create_wall_timer(10ms, timer_callback);

        auto asif_activate_callback = [this]() -> void {
            mav1_des_ << 0.0, 0.0, -2.0, 0.0; //x, y, z, yaw
            mav2_des_ << 0.0, 0.0, -2.0, 0.0; //x, y, z, yaw
            asif_active_ = true;
            asif_activate_timer_->cancel();
            asif_activate_timer_->reset();
        };

        asif_activate_timer_ = this->create_wall_timer(30s, asif_activate_callback);
    }

    void arm() const;

    void disarm() const;

private:
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
    double safe_displacement_y_;
    double safe_displacement_z_;
    double quad1_max_thrust_body_;
    double quad2_max_thrust_body_;
    double quad1_min_thrust_body_;
    double quad2_min_thrust_body_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr asif_activate_timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<VehicleOdometry>::SharedPtr mav1_estimator_odometry_subscriber_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr mav2_estimator_odometry_subscriber_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;

    std::atomic<uint64_t> timestamp_;

    uint64_t offboard_setpoint_counter_;

    ASIF asif_solver_;
    bool asif_active_{false};

    VehicleOdometry mav1_;
    VehicleOdometry mav2_;
    VehicleRatesSetpoint control1_;
    VehicleRatesSetpoint control2_;
    Eigen::Vector4d mav1_des_;
    Eigen::Vector4d mav2_des_;

    std::string mav1_namespace_ = "/mav1/";
    std::string mav2_namespace_ = "/mav2/";

    void geometric_controller(const Eigen::Vector4d &mav1_des,
                              const Eigen::Vector4d &mav2_des,
                              VehicleRatesSetpoint &control1,
                              VehicleRatesSetpoint &control2);

    void publish_offboard_control_mode() const;

    void publish_vehicle_command(uint16_t command, float param1 = 0.0,
                                 float param2 = 0.0) const;
};

void Mav2ASIF::geometric_controller(const Eigen::Vector4d &mav1_des,
                                    const Eigen::Vector4d &mav2_des,
                                    VehicleRatesSetpoint &control1,
                                    VehicleRatesSetpoint &control2) {
    double f_cmd_x1 = spring_constantX_ * tanh(spring_saturation_ * (mav1_des.x() - mav1_.x)) -
                      spring_dampenerX_ * mav1_.vx;
    double f_cmd_x2 = spring_constantX_ * tanh(spring_saturation_ * (mav2_des.x() - mav2_.x)) -
                      spring_dampenerX_ * mav2_.vx;
    double f_cmd_y1 = spring_constantYZ_ * tanh(spring_saturation_ * (mav1_des.y() - mav1_.y)) -
                      spring_dampenerYZ_ * mav1_.vy;
    double f_cmd_y2 = spring_constantYZ_ * tanh(spring_saturation_ * (mav2_des.y() - mav2_.y)) -
                      spring_dampenerYZ_ * mav2_.vy;
    double f_cmd_z1 = spring_constantYZ_ * tanh(spring_saturation_ * (mav1_des.z() - mav1_.z)) - mass1_ * gravity_ -
                      spring_dampenerYZ_ * mav1_.vz;
    double f_cmd_z2 = spring_constantYZ_ * tanh(spring_saturation_ * (mav2_des.z() - mav2_.z)) - mass2_ * gravity_ -
                      spring_dampenerYZ_ * mav2_.vz;

    Eigen::Vector3d thrust_dir1{f_cmd_x1, f_cmd_y1, f_cmd_z1};
    Eigen::Vector3d thrust_dir2{f_cmd_x2, f_cmd_y2, f_cmd_z2};
    thrust_dir1.normalize();
    thrust_dir2.normalize();

    Eigen::Vector3d eulers1 = quaternion_to_rpy_wrap(Eigen::Quaterniond(mav1_.q[0],
                                                                        mav1_.q[1],
                                                                        mav1_.q[2],
                                                                        mav1_.q[3]));
    Eigen::Vector3d eulers2 = quaternion_to_rpy_wrap(Eigen::Quaterniond(mav2_.q[0],
                                                                        mav2_.q[1],
                                                                        mav2_.q[2],
                                                                        mav2_.q[3]));

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


//	std::cout << "--------" << std::endl;
//	std::cout << iterm_thrust_frame1_1 << std::endl;
//	std::cout << thrust_dir1 << std::endl;
//	std::cout << eulers1 << std::endl;
//	std::cout << phi1_cmd << std::endl;
//	std::cout << theta1_cmd << std::endl;
    control1.thrust_body[2] = -(-f_cmd_x1 * cos(eulers1.x()) * sin(eulers1.y()) + f_cmd_y1 * sin(eulers1.x())
                                - f_cmd_z1 * cos(eulers1.x()) * cos(eulers1.y()) - quad1_min_thrust_body_) / (quad1_max_thrust_body_ - quad1_min_thrust_body_);
    control2.thrust_body[2] = -(-f_cmd_x2 * cos(eulers2.x()) * sin(eulers2.y()) + f_cmd_y2 * sin(eulers2.x())
                                - f_cmd_z2 * cos(eulers2.x()) * cos(eulers2.y()) - quad2_min_thrust_body_) / (quad2_max_thrust_body_ - quad2_min_thrust_body_);

    control1.roll = -roll_kp_ * (eulers1.x() - phi1_cmd);
    control2.roll = -roll_kp_ * (eulers2.x() - phi2_cmd);

    control1.pitch = -pitch_yaw_kp_ * (eulers1.y() - theta1_cmd);
    control2.pitch = -pitch_yaw_kp_ * (eulers2.y() - theta2_cmd);

    control1.yaw = -pitch_yaw_kp_ * (eulers1.z() - mav1_des(3));
    control2.yaw = -pitch_yaw_kp_ * (eulers2.z() - mav2_des(3));
}


/**
 * @brief Send a command to Arm the vehicle
 */
void Mav2ASIF::arm() const {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void Mav2ASIF::disarm() const {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void Mav2ASIF::publish_offboard_control_mode() const {
    OffboardControlMode msg{};
    msg.timestamp = timestamp_.load();
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = true;

    offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void Mav2ASIF::publish_vehicle_command(uint16_t command, float param1,
                                       float param2) const {
    VehicleCommand msg{};
    msg.timestamp = timestamp_.load();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 2;
    msg.target_component = 1;
    msg.source_system = 2;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[]) {
    std::cout << "Starting MAV2 ASIF example..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mav2ASIF>());

    rclcpp::shutdown();
    return 0;
}

