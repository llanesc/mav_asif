#pragma once

#include "Eigen/Core"
#include <rclcpp/rclcpp.hpp>
#include "osqp.h"
#include "workspace.h"
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <queue>

namespace asif
{

#define NUM_STATES 8
#define NUM_CONTROL_INPUTS 4
#define NUM_DISTURBANCES 6
#define POWER_OF_TWO(EXPONENT) (1 << (EXPONENT))

//typedef struct {
//	double x;
//	double y;
//	double z;
//	double vx;
//	double vy;
//	double vz;
//	double phi;
//	double theta;
//	double psi;
//} quad_states_s;
//
//typedef struct {
//	double thrust;
//	double roll_rate;
//} control_s;


typedef Eigen::Matrix<double, 8, 1> state_vector_t;
typedef Eigen::Matrix<double, 8, 8> state_jacobian_t;

class embeddingState: public Eigen::Matrix<double, 16, 1> {
public:
	// Constructors
	embeddingState()
			: Eigen::Matrix<double, 16, 1>()
	{};

	typedef Eigen::Matrix<double, 16, 1> Base;

    embeddingState(const state_vector_t &x, const state_vector_t &xh)
			: Eigen::Matrix<double, 16, 1>()
	{
		Eigen::Matrix<double, 16, 1> other;
		other << x, xh;
		this->Base::operator=(other);
	};

	embeddingState(const Eigen::Matrix<double, 16, 1> &other)
			: Eigen::Matrix<double, 16, 1>(other)
	{
	};


	//Operators
	embeddingState &operator=(const Eigen::Matrix<double, 16, 1> &other)
	{
		this->Base::operator=(other);
		return *this;
	}

	template<typename OtherDerived>
	Eigen::Product<embeddingState, OtherDerived> operator*(const MatrixBase<OtherDerived> &other) const
	{
		this->Base::operator*(other);
		return *this;
	}

	//get content helpers
    Eigen::Matrix<double, 8, 1> get_x()
	{
        Eigen::Matrix<double, 8, 1> x = this->Base::DenseBase::operator()(Eigen::seq(0, 7, 1));
        return x;
	}

    Eigen::Matrix<double, 8, 1> get_xh()
	{
        Eigen::Matrix<double, 8, 1> xh = this->Base::DenseBase::operator()(Eigen::seq(8, 15, 1));
        return xh;
	}

    Eigen::Matrix<double, 8, 1> get_x() const
	{
        Eigen::Matrix<double, 8, 1> x = this->Base::DenseBase::operator()(Eigen::seq(0, 7, 1));
        return x;
	}

    Eigen::Matrix<double, 8, 1> get_xh() const
	{
        Eigen::Matrix<double, 8, 1> xh = this->Base::DenseBase::operator()(Eigen::seq(8, 15, 1));
        return xh;
	}

	double get_x(int row)
	{
	    return (*this)[row];
	}

	double get_xh(int row)
	{
        return (*this)[row + 8];
	}

	double get_x(int row) const
	{
        return (*this)[row];
	}

	double get_xh(int row) const
	{
        return (*this)[row + 8];
	}
};

typedef Eigen::Matrix<double, 6, 1> disturbance_vector_t;
typedef Eigen::Matrix<double, 1, 16> embedding_state_gradient_t;

class ASIF: public rclcpp::Node {
public:
	ASIF()
			: Node("asif_node")
	{
		try {
			std::vector<double> lyapunov_function_P_matrix_tmp(lyapunov_function_P_matrix_.size());
			std::vector<double> w_min_tmp(w_min_.size());
			std::vector<double> w_max_tmp(w_max_.size());

			this->declare_parameter("mass1");
			this->declare_parameter("mass2");
			this->declare_parameter("gravity");
			this->declare_parameter("safe_displacement.y");
			this->declare_parameter("safe_displacement.z");
			this->declare_parameter("backup_dynamics.spring_constant");
			this->declare_parameter("backup_dynamics.spring_dampener");
			this->declare_parameter("backup_dynamics.spring_saturation");
			this->declare_parameter("backup_dynamics.roll_P");
			this->declare_parameter("quad1_max_thrust_body");
			this->declare_parameter("quad2_max_thrust_body");
			this->declare_parameter("disturbance_min");
			this->declare_parameter("disturbance_max");
			this->declare_parameter("lyapunov_function_P_matrix");
			this->declare_parameter("barrier_function_constant");
			this->declare_parameter("simulation.backup_horizon");
			this->declare_parameter("simulation.dt_backup");
			this->declare_parameter("log_distribution_parameters.log_distribution_steps");
			this->declare_parameter("log_distribution_parameters.spacing_factor");
			this->declare_parameter("log_distribution_parameters.corner_power");
			this->declare_parameter("soft_min_parameters.fitness_multiplier");

			this->get_parameter("mass1", mass1_);
			this->get_parameter("mass2", mass2_);
			this->get_parameter("gravity", gravity_);
			this->get_parameter("asif_alpha", asif_alpha_);
			this->get_parameter("safe_displacement.y", safe_displacement_y_);
			this->get_parameter("safe_displacement.z", safe_displacement_z_);
			this->get_parameter("backup_dynamics.spring_constant", spring_constant_);
			this->get_parameter("backup_dynamics.spring_dampener", spring_dampener_);
			this->get_parameter("backup_dynamics.spring_saturation", spring_saturation_);
			this->get_parameter("backup_dynamics.roll_P", k_roll_P_);
			this->get_parameter("quad1_max_thrust_body", quad1_max_thrust_body_);
			this->get_parameter("quad2_max_thrust_body", quad2_max_thrust_body_);
			this->get_parameter("disturbance_min", w_min_tmp);
			this->get_parameter("disturbance_max", w_max_tmp);
			this->get_parameter("lyapunov_function_P_matrix", lyapunov_function_P_matrix_tmp);
			this->get_parameter("barrier_function_constant", barrier_function_constant);
			this->get_parameter("simulation.backup_horizon", backup_horizon_);
			this->get_parameter("simulation.dt_backup", dt_backup_);
			this->get_parameter("log_distribution_parameters.log_distribution_steps", log_distribution_steps_);
			this->get_parameter("log_distribution_parameters.spacing_factor", spacing_factor_);
			this->get_parameter("log_distribution_parameters.corner_power", corner_power_);
			this->get_parameter("soft_min_parameters.fitness_multiplier", soft_min_fitness_multiplier_);
			lyapunov_function_P_matrix_ = Eigen::Map<Eigen::Matrix<double,
			                                                       8,
			                                                       8>>(lyapunov_function_P_matrix_tmp.data());
			w_min_ = Eigen::Map<disturbance_vector_t, Eigen::Unaligned>(w_min_tmp.data());
			w_max_ = Eigen::Map<disturbance_vector_t, Eigen::Unaligned>(w_max_tmp.data());

		} catch (rclcpp::ParameterTypeException &excp) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Parameter type exception caught");
			rclcpp::shutdown(nullptr, "ASIF ERROR: Parameter type exception caught on initialization");
		}

		xbar_ << 0., 0., 0., 0., 0., 0., safe_displacement_y_, safe_displacement_z_;
	};

	~ASIF() override = default;

	int QP(OSQPWorkspace *osqp_workspace, const px4_msgs::msg::VehicleOdometry &mav1,
	       const px4_msgs::msg::VehicleOdometry &mav2,
	       px4_msgs::msg::VehicleRatesSetpoint *controls1,
	       px4_msgs::msg::VehicleRatesSetpoint *controls2);

private:
	double gravity_;
	double mass1_;
	double mass2_;
	double spring_saturation_;
	double spring_constant_;
	double spring_dampener_;
	double asif_alpha_;
	double k_roll_P_;
	double safe_displacement_y_;
	double safe_displacement_z_;
	double quad1_max_thrust_body_;
	double quad2_max_thrust_body_;

	disturbance_vector_t w_min_;
	disturbance_vector_t w_max_;

	state_vector_t xbar_;

	Eigen::Matrix<double, 8, 8> lyapunov_function_P_matrix_;
	double barrier_function_constant;

	double dt_backup_;
	int backup_horizon_; // number of backup steps of dt_backup_ backup_time = T_backup_*dt_backup_
	int log_distribution_steps_;
	int max_barrier_index_in_backup_horizon_{-1};

	bool QP_initialized = false;

	//log distribution algorithm parameters
	double spacing_factor_;
	int corner_power_; //must be odd

	//Soft min parameters
	double soft_min_fitness_multiplier_;

	embeddingState embedding_state_trajectory_;

	embeddingState embedding_state_update(const embeddingState &embedding_state) const;

	state_vector_t decomposition_function(const embeddingState &embedding_state, const disturbance_vector_t &w) const;

	embeddingState embedding_dynamics(const embeddingState &phi) const;

	state_jacobian_t jacobian(state_vector_t x) const;

	double barrier_function(const state_vector_t& x) const;

	void backupTrajectoryDistribution(const int &traj_t, const int &backup_steps, std::queue<int> &out) const;

	void barrier_soft_min(const embeddingState &embedding_state_trajectory,
	                      std::queue<double> &soft_min,
	                      embedding_state_gradient_t *soft_min_gradient) const;

}; //class ASIF

} //namespace asif