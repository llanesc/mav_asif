#include "mav_asif_util.hpp"
#include <chrono>
#include <iostream>
#include "mav_util.h"

using namespace asif;
using namespace Eigen;

int ASIF::QP(OSQPWorkspace *osqp_workspace, const px4_msgs::msg::VehicleOdometry &mav1,
             const px4_msgs::msg::VehicleOdometry &mav2,
             mavControl &mav1_control,
             mavControl &mav2_control,
             double &worst_barrier,
             double &worst_barrier_time) {
    const std::chrono::time_point<std::chrono::steady_clock> start =
            std::chrono::steady_clock::now();
    Matrix<double, NUM_STATES, 1> f_x;
    Matrix<double, NUM_STATES, NUM_CONTROL_INPUTS> g_x;

    Eigen::Vector3d eulers1 = quaternion_to_rpy_wrap(Eigen::Quaterniond(mav1.q[0],
                                                                        mav1.q[1],
                                                                        mav1.q[2],
                                                                        mav1.q[3]));
    Eigen::Vector3d eulers2 = quaternion_to_rpy_wrap(Eigen::Quaterniond(mav2.q[0],
                                                                        mav2.q[1],
                                                                        mav2.q[2],
                                                                        mav2.q[3]));

    f_x.setZero();
    f_x(2) = gravity_;
    f_x(3) = gravity_;
    f_x(4) = 0;
    f_x(5) = 0;
    f_x(6) = mav2.vy - mav1.vy;
    f_x(7) = mav2.vz - mav1.vz;

    g_x.setZero();
    g_x(0, 0) = sin(eulers1.x()) / mass1_;
    g_x(1, 1) = sin(eulers2.x()) / mass2_;
    g_x(2, 0) = -cos(eulers1.x()) / mass1_;
    g_x(3, 1) = -cos(eulers2.x()) / mass2_;
    g_x(4, 2) = 1;
    g_x(5, 3) = 1;

    state_vector_t x;
    x(0) = mav1.vy;
    x(1) = mav2.vy;
    x(2) = mav1.vz;
    x(3) = mav2.vz;
    x(4) = eulers1.x();
    x(5) = eulers2.x();
    x(6) = mav2.y - mav1.y;
    x(7) = mav2.z - mav1.z;
    embedding_state_trajectory_ = embeddingState(x, x);

    std::queue<double> hs_min_backup;
    std::queue<int> backup_times_backup;

    if (QP_initialized && max_barrier_index_in_backup_horizon_ != -1) {
        backupTrajectoryDistribution(max_barrier_index_in_backup_horizon_, backup_horizon_, backup_times_backup);
    } else {
        for (int i = 0; i < log_distribution_steps_; i++) {
            backup_times_backup.push(static_cast<int>(i * (backup_horizon_) / (log_distribution_steps_ - 1)));
        }
    }

    // Backup set max of min
    max_barrier_index_in_backup_horizon_ = 0;
    double Psi_backup = -INFINITY;
    embeddingState worst_embedding_state;
    state_jacobian_t Q1;
    Q1.setIdentity();
    state_jacobian_t Q2;
    Q2.setIdentity();
    Matrix<double, 2 * NUM_STATES, NUM_STATES> QMatrix_backup;

    for (int i = 0; i < backup_horizon_; i++) {
        if (i == backup_times_backup.front()) {

            barrier_soft_min(embedding_state_trajectory_, hs_min_backup, nullptr);

            if (hs_min_backup.back() > Psi_backup) {
                Psi_backup = hs_min_backup.back();
                max_barrier_index_in_backup_horizon_ = i;
                worst_embedding_state = embedding_state_trajectory_;
                QMatrix_backup << Q1,
                                  Q2;
            }
	        backup_times_backup.pop();
        }
        Q1 = jacobian(embedding_state_trajectory_.get_x()) * Q1 * dt_backup_ + Q1;
        Q2 = jacobian(embedding_state_trajectory_.get_xh()) * Q2 * dt_backup_ + Q2;
        embedding_state_trajectory_ = embedding_state_update(embedding_state_trajectory_);
    }

    embedding_state_gradient_t logsumexp_gradient;
    barrier_soft_min(worst_embedding_state, hs_min_backup, &logsumexp_gradient);

    Matrix<double, 1, NUM_STATES> DPsi_backup = logsumexp_gradient * QMatrix_backup;

    double q_new[NUM_CONTROL_INPUTS] = {-mav1_control.thrust,
                                        -mav2_control.thrust,
                                        -mav1_control.roll_rate,
                                        -mav2_control.roll_rate};
    double warm_x[NUM_CONTROL_INPUTS] = {mav1_control.thrust,
									     mav2_control.thrust,
									     mav1_control.roll_rate,
									     mav2_control.roll_rate};
    double ub_new[POWER_OF_TWO(NUM_DISTURBANCES)];
    double A_new[NUM_CONTROL_INPUTS * POWER_OF_TWO(NUM_DISTURBANCES)];

    Matrix<double, 1, NUM_CONTROL_INPUTS> Ax_backup(-DPsi_backup * g_x);

    Vector<double, NUM_STATES> w_temp;
    for (int k = 0; k < POWER_OF_TWO(NUM_DISTURBANCES); k++) {
        for (int j = 0; j < NUM_DISTURBANCES; j++) {
            if ((k & (0b000001 << j)) == (0b000001 << j)) {
                w_temp(j) = w_max_(j);
            } else {
                w_temp(j) = w_min_(j);
            }
        }
        ub_new[k] = (DPsi_backup * (f_x + w_temp) + asif_alpha_ * copysign(1.0, Psi_backup) * pow(abs(Psi_backup),
                                                                                                  1.0));
    }

    for (int Axindx = 0; Axindx < NUM_CONTROL_INPUTS * POWER_OF_TWO(NUM_DISTURBANCES); Axindx++) {
        A_new[Axindx] = Ax_backup(0, Axindx / POWER_OF_TWO(NUM_DISTURBANCES));
    }

    if (osqp_update_A(osqp_workspace, A_new, OSQP_NULL, osqp_workspace->data->A->nzmax)) {
        return -1;
    }
    if (osqp_update_upper_bound(osqp_workspace, ub_new)) {
        return -1;
    }
    if (osqp_update_lin_cost(osqp_workspace, q_new)) {
        return -1;
    }
    if (osqp_warm_start_x(osqp_workspace, warm_x)) {
        return -1;
    }
    if (osqp_solve(osqp_workspace)) {
        return -1;
    }

	mav1_control.thrust = osqp_workspace->solution->x[0];
	mav1_control.roll_rate = osqp_workspace->solution->x[2];

	mav2_control.thrust = osqp_workspace->solution->x[1];
	mav2_control.roll_rate = osqp_workspace->solution->x[3];

    printf("Status:                %s\n", (osqp_workspace)->info->status);
    printf("Number of iterations:  %d\n", (int) ((osqp_workspace)->info->iter));
    printf("obj_val:               %f\n", (osqp_workspace)->info->obj_val);
    printf("prim_res:              %f\n", (osqp_workspace)->info->pri_res);

    printf("Tau1: %f, Tau2: %f, Mx1: %f, Mx2: %f \n", osqp_workspace->solution->x[0],
           osqp_workspace->solution->x[1],
           osqp_workspace->solution->x[2],
           osqp_workspace->solution->x[3]);
    (void) printf("Psi_backup:                          %.10f\n", Psi_backup);

    int solution_solved = (OSQP_SOLVED == osqp_workspace->info->status_val);

    if (solution_solved) {
        QP_initialized = true;
    }

    worst_barrier = Psi_backup;
    worst_barrier_time = max_barrier_index_in_backup_horizon_*dt_backup_;

    const auto end = std::chrono::steady_clock::now();
    std::cout << "QP calculations took " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << "µs\n";

    return solution_solved;
}

double ASIF::barrier_function(const state_vector_t& x) const {

    return barrier_function_constant - (x - xbar_).transpose() * lyapunov_function_P_matrix_ * (x - xbar_);
}

void ASIF::barrier_soft_min(const embeddingState &embedding_state_trajectory,
                            std::queue<double> &soft_min,
                            embedding_state_gradient_t *soft_min_gradient) const {
    Matrix<double, NUM_STATES, POWER_OF_TWO(NUM_STATES)> reachable_set_corners;
    Matrix<double, POWER_OF_TWO(NUM_STATES), 1> barrier_function_corners;
    int max_indx = 0;

    for (int i = 0; i < POWER_OF_TWO(NUM_STATES); i++) {
        for (int j = 0; j < NUM_STATES; j++) {
            if ((i & (0b00000001 << j)) == (0b00000001 << j)) {
                reachable_set_corners(j, i) = embedding_state_trajectory.get_x(j);
            } else {
                reachable_set_corners(j, i) = embedding_state_trajectory.get_xh(j);
            }
        }

        barrier_function_corners(i) = barrier_function(reachable_set_corners.col(i));

        if (barrier_function_corners(i) > barrier_function_corners(max_indx)) {
            max_indx = i;
        }
    }

    double xstar = barrier_function_corners(max_indx) * soft_min_fitness_multiplier_;

    barrier_function_corners = (barrier_function_corners.array() * soft_min_fitness_multiplier_ - xstar).exp();
    double sum_of_exp = barrier_function_corners.sum();

    // If we need to compute gradient
    if (soft_min_gradient) {
        Matrix<double, NUM_STATES, 2 * NUM_STATES> binary_matrix;
        binary_matrix.setZero();
        soft_min_gradient->setZero();
        for (int i = 0; i < POWER_OF_TWO(NUM_STATES); i++) {
            for (int j = 0; j < NUM_STATES; j++) {
                if ((i & (0b00000001 << j)) == (0b00000001 << j)) {
                    binary_matrix(j, j) = 0.0;
                    binary_matrix(j, j + NUM_STATES) = 1.0;
                } else {
                    binary_matrix(j, j) = 1.0;
                    binary_matrix(j, j + NUM_STATES) = 0.0;
                }
            }
            *soft_min_gradient += barrier_function_corners(i) *
                                  (-2.0 * (reachable_set_corners.col(i) - xbar_).transpose() *
                                   (lyapunov_function_P_matrix_)) * binary_matrix;
        }
        *soft_min_gradient = *soft_min_gradient / sum_of_exp;
    } else {
        soft_min.push(1.0 / soft_min_fitness_multiplier_ * (xstar + log(sum_of_exp)));
    }
}

state_jacobian_t ASIF::jacobian(state_vector_t x) const {
    state_jacobian_t J;
    double dy_tanh = tanh(spring_saturation_ * (x(6) - safe_displacement_y_));
    double dz_tanh = tanh(spring_saturation_ * (x(7) - safe_displacement_z_));
    double fy1 = spring_constant_ * dy_tanh - spring_dampener_ * x(0);
    double fy2 = -spring_constant_ * dy_tanh - spring_dampener_ * x(1);
    double fz1 = spring_constant_ * dz_tanh - mass1_ * gravity_ - spring_dampener_ * x(2);
    double fz2 = -spring_constant_ * dz_tanh - mass2_ * gravity_ - spring_dampener_ * x(3);
    double fnorm1 = sqrt(pow(fy1, 2.0) + pow(fz1, 2.0));
    double fnorm2 = sqrt(pow(fy2, 2.0) + pow(fz2, 2.0));

    J(0, 0) = -spring_dampener_ / mass1_ * pow(sin(x(4)), 2.0);
    J(0, 1) = 0.0;
    J(0, 2) = spring_dampener_ / mass1_ * sin(x(4)) * cos(x(4));
    J(0, 3) = 0.0;
    J(0, 4) = 1.0 / mass1_ * (fy1 * sin(2 * x(4)) - fz1 * cos(2.0 * x(4)));
    J(0, 5) = 0.0;
    J(0, 6) = spring_constant_ * spring_saturation_ / mass1_ * pow(sin(x(4)), 2.0) * (1.0 - pow(dy_tanh, 2.0));
    J(0, 7) = -spring_constant_ * spring_saturation_ / mass1_ * sin(x(4)) * cos(x(4)) * (1.0 - pow(dz_tanh, 2.0));

    J(1, 0) = 0.0;
    J(1, 1) = -spring_dampener_ / mass2_ * pow(sin(x(5)), 2.0);
    J(1, 2) = 0.0;
    J(1, 3) = spring_dampener_ / mass2_ * sin(x(5)) * cos(x(5));
    J(1, 4) = 0.0;
    J(1, 5) = 1.0 / mass2_ * (fy2 * sin(2.0 * x(5)) - fz2 * cos(2.0 * x(5)));
    J(1, 6) = -spring_constant_ * spring_saturation_ / mass2_ * pow(sin(x(5)), 2) * (1. - pow(dy_tanh, 2.0));
    J(1, 7) = spring_constant_ * spring_saturation_ / mass2_ * sin(x(5)) * cos(x(5)) * (1. - pow(dz_tanh, 2.0));

    J(2, 0) = spring_dampener_ / mass1_ * sin(x(4)) * cos(x(4));
    J(2, 1) = 0.0;
    J(2, 2) = -spring_dampener_ / mass1_ * pow(cos(x(4)), 2.0);
    J(2, 3) = 0.0;
    J(2, 4) = -1.0 / mass1_ * (fy1 * cos(2.0 * x(4)) + fz1 * sin(2.0 * x(4)));
    J(2, 5) = 0.0;
    J(2, 6) = -spring_constant_ * spring_saturation_ / mass1_ * sin(x(4)) * cos(x(4)) * (1 - pow(dy_tanh, 2.0));
    J(2, 7) = spring_constant_ * spring_saturation_ / mass1_ * pow(cos(x(4)), 2) * (1 - pow(dz_tanh, 2.0));

    J(3, 0) = 0.0;
    J(3, 1) = spring_dampener_ / mass2_ * sin(x(5)) * cos(x(5));
    J(3, 2) = 0.0;
    J(3, 3) = -spring_dampener_ / mass2_ * pow(cos(x(5)), 2.0);
    J(3, 4) = 0.0;
    J(3, 5) = -1.0 / mass2_ * (fy2 * cos(2 * x(5)) + fz2 * sin(2 * x(5)));
    J(3, 6) = spring_constant_ * spring_saturation_ / mass2_ * sin(x(5)) * cos(x(5)) * (1 - pow(dy_tanh, 2.0));
    J(3, 7) = -spring_constant_ * spring_saturation_ / mass2_ * pow(cos(x(5)), 2) * (1 - pow(dz_tanh, 2.0));

    J(4, 0) = -(fz1 * k_roll_P_ * spring_dampener_ * (fz1 * cos(x(4)) - fy1 * sin(x(4)))) / pow(fnorm1, 3.0);
    J(4, 1) = 0.0;
    J(4, 2) = (fy1 * k_roll_P_ * spring_dampener_ * (fz1 * cos(x(4)) - fy1 * sin(x(4)))) / pow(fnorm1, 3.0);
    J(4, 3) = 0.0;
    J(4, 4) = (k_roll_P_ * (fz1 * cos(x(4)) - fy1 * sin(x(4)))) / fnorm1;
    J(4, 5) = 0.0;
    J(4, 6) = (2.0 * fz1 * k_roll_P_ * spring_constant_ * spring_saturation_ * (fz1 * cos(x(4)) - fy1 * sin(x(4)))) /
              ((cosh(
                      2.0 * spring_saturation_ * (x(6) - safe_displacement_y_)) + 1) * pow(fnorm1, 3.0));
    J(4, 7) = -(2.0 * fy1 * k_roll_P_ * spring_constant_ * spring_saturation_ * (fz1 * cos(x(4)) - fy1 * sin(x(4)))) /
              ((cosh(
                      2.0 * spring_saturation_ * (x(7) - safe_displacement_z_)) + 1) * pow(fnorm1, 3.0));

    J(5, 0) = 0.0;
    J(5, 1) = -(fz2 * k_roll_P_ * spring_dampener_ * (fz2 * cos(x(5)) - fy2 * sin(x(5)))) / pow(fnorm2, 3.0);
    J(5, 2) = 0.0;
    J(5, 3) = (fy2 * k_roll_P_ * spring_dampener_ * (fz2 * cos(x(5)) - fy2 * sin(x(5)))) / pow(fnorm2, 3.0);
    J(5, 4) = 0.0;
    J(5, 5) = (k_roll_P_ * (fz2 * cos(x(5)) - fy2 * sin(x(5)))) / fnorm2;
    J(5, 6) = -(2.0 * fz2 * k_roll_P_ * spring_constant_ * spring_saturation_ * (fz2 * cos(x(5)) - fy2 * sin(x(5)))) /
              ((cosh(
                      2.0 * spring_saturation_ * (x(6) - safe_displacement_y_)) + 1) * pow(fnorm2, 3.0));
    J(5, 7) = (2.0 * fy2 * k_roll_P_ * spring_constant_ * spring_saturation_ * (fz2 * cos(x(5)) - fy2 * sin(x(5)))) /
              ((cosh(
                      2.0 * spring_saturation_ * (x(7) - safe_displacement_z_)) + 1) * pow(fnorm2, 3.0));

    J(6, 0) = -1.0;
    J(6, 1) = 1.0;
    J(6, 2) = 0.0;
    J(6, 3) = 0.0;
    J(6, 4) = 0.0;
    J(6, 5) = 0.0;
    J(6, 6) = 0.0;
    J(6, 7) = 0.0;


    J(7, 0) = 0.0;
    J(7, 1) = 0.0;
    J(7, 2) = -1.0;
    J(7, 3) = 1.0;
    J(7, 4) = 0.0;
    J(7, 5) = 0.0;
    J(7, 6) = 0.0;
    J(7, 7) = 0.0;

    return J;
}

embeddingState ASIF::embedding_state_update(const embeddingState &embedding_state) const {

    embeddingState embedding_state_new;

    embedding_state_new = embedding_state + dt_backup_ * embedding_dynamics(embedding_state);

    return embedding_state_new;
}

embeddingState ASIF::embedding_dynamics(const embeddingState &embedding_state) const {
    embeddingState flipped_embedding_state(embedding_state.get_xh(), embedding_state.get_x());
    embeddingState embedding_state_derivative(decomposition_function(embedding_state, w_min_),
                                              decomposition_function(flipped_embedding_state, w_max_));

    return embedding_state_derivative;
}

state_vector_t
ASIF::decomposition_function(const embeddingState &embedding_state, const disturbance_vector_t &w) const {
    state_vector_t embedding_dynamics;
    double y1;
    double y2;
    double y3;
    double y4;
    double y7;
    double y8;
    double za;
    double zb;
    double theta_s;
    double theta_ss;
    double J;
    double q;
    double q_under;
    double q_over;

    // ------------------------- D1 ----------------------------------------

    if (sin(embedding_state.get_x(4)) * cos(embedding_state.get_x(4)) >= 0.) {
        y3 = embedding_state.get_x(2);
        y8 = embedding_state.get_xh(7);
    } else {
        y3 = embedding_state.get_xh(2);
        y8 = embedding_state.get_x(7);
    }

    za = spring_constant_ / mass1_ * tanh(spring_saturation_ * (embedding_state.get_x(6) - safe_displacement_y_)) -
         spring_dampener_ / mass1_ * embedding_state.get_x(
                 0);
    zb = -spring_constant_ / mass1_ * tanh(spring_saturation_ * (y8 - safe_displacement_z_)) + gravity_ +
         spring_dampener_ / mass1_ * y3;

    theta_s = 0.5 * atan2(-zb, za);
    theta_ss = 0.5 * atan2(za, zb);

    J = za * sin(2.0 * theta_ss) + zb * cos(2.0 * theta_ss);

    q = za * pow(sin(embedding_state.get_x(4)),
                 2.0) + zb * cos(embedding_state.get_x(4)) * sin(embedding_state.get_x(4)) + J * (embedding_state.get_x(
            4) - embedding_state.get_xh(4));
    q_under = za * pow(sin(theta_s), 2.0) + zb / 2.0 * sin(2.0 * theta_s);
    q_over = za * pow(cos(theta_s), 2.0) - zb / 2.0 * sin(2.0 * theta_s);

    embedding_dynamics(0) = fmin(q_over, fmax(q_under, q)) + w(0);

    // ------------------------- D2 ----------------------------------------

    if (sin(embedding_state.get_x(5)) * cos(embedding_state.get_x(5)) >= 0.) {
        y4 = embedding_state.get_x(3);
        y8 = embedding_state.get_x(7);
    } else {
        y4 = embedding_state.get_xh(3);
        y8 = embedding_state.get_xh(7);
    }

    za = -spring_constant_ / mass2_ * tanh(spring_saturation_ * (embedding_state.get_xh(6) - safe_displacement_y_)) -
         spring_dampener_ / mass2_ * embedding_state.get_x(
                 1);
    zb = spring_constant_ / mass2_ * tanh(spring_saturation_ * (y8 - safe_displacement_z_)) + gravity_ +
         spring_dampener_ / mass2_ * y4;

    theta_s = 0.5 * atan2(-zb, za);
    theta_ss = 0.5 * atan2(za, zb);
    J = za * sin(2.0 * theta_ss) + zb * cos(2.0 * theta_ss);

    q = za * pow(sin(embedding_state.get_x(5)),
                 2.0) + zb * cos(embedding_state.get_x(5)) * sin(embedding_state.get_x(5)) + J * (embedding_state.get_x(
            5) - embedding_state.get_xh(5));
    q_under = za * pow(sin(theta_s), 2.0) + zb / 2.0 * sin(2.0 * theta_s);
    q_over = za * pow(cos(theta_s), 2.0) - zb / 2.0 * sin(2.0 * theta_s);

    embedding_dynamics(1) = fmin(q_over, fmax(q_under, q)) + w(1);

    // ------------------------- D3 ----------------------------------------

    if (sin(embedding_state.get_x(4)) * cos(embedding_state.get_x(4)) >= 0.) {
        y1 = embedding_state.get_x(0);
        y7 = embedding_state.get_xh(6);
    } else {
        y1 = embedding_state.get_xh(0);
        y7 = embedding_state.get_x(6);
    }

    za = spring_constant_ / mass1_ * tanh(spring_saturation_ * (embedding_state.get_x(7) - safe_displacement_z_)) -
         gravity_ - spring_dampener_ / mass1_ * embedding_state.get_x(
            2);
    zb = -spring_constant_ / mass1_ * tanh(spring_saturation_ * (y7 - safe_displacement_y_)) +
         spring_dampener_ / mass1_ * y1;

    theta_s = 0.5 * atan2(zb, za);
    theta_ss = 0.5 * atan2(-za, zb);

    J = zb * cos(2.0 * theta_ss) - za * sin(2.0 * theta_ss);

    q = za * pow(cos(embedding_state.get_x(4)),
                 2.0) + zb * cos(embedding_state.get_x(4)) * sin(embedding_state.get_x(4)) + gravity_ +
        J * (embedding_state.get_x(
                4) - embedding_state.get_xh(4));
    q_under = za * pow(sin(theta_s), 2.0) - zb / 2.0 * sin(2.0 * theta_s) + gravity_;
    q_over = za * pow(cos(theta_s), 2.0) + zb / 2.0 * sin(2.0 * theta_s) + gravity_;

    embedding_dynamics(2) = fmin(q_over, fmax(q_under, q)) + w(2);

    // ------------------------- D4 ----------------------------------------

    if (sin(embedding_state.get_x(5)) * cos(embedding_state.get_x(5)) >= 0.) {
        y2 = embedding_state.get_x(1);
        y7 = embedding_state.get_x(6);
    } else {
        y2 = embedding_state.get_xh(1);
        y7 = embedding_state.get_xh(6);
    }

    za = -spring_constant_ / mass2_ * tanh(spring_saturation_ * (embedding_state.get_xh(7) - safe_displacement_z_)) -
         gravity_ - spring_dampener_ / mass2_ * embedding_state.get_x(
            3);
    zb = spring_constant_ / mass2_ * tanh(spring_saturation_ * (y7 - safe_displacement_y_)) +
         spring_dampener_ / mass2_ * y2;

    theta_s = 0.5 * atan2(zb, za);
    theta_ss = 0.5 * atan2(-za, zb);
    J = zb * cos(2.0 * theta_ss) - za * sin(2.0 * theta_ss);

    q = za * pow(cos(embedding_state.get_x(5)),
                 2.0) + zb * cos(embedding_state.get_x(5)) * sin(embedding_state.get_x(5)) + gravity_ +
        J * (embedding_state.get_x(
                5) - embedding_state.get_xh(5));
    q_under = za * pow(sin(theta_s), 2.0) - zb / 2.0 * sin(2.0 * theta_s) + gravity_;
    q_over = za * pow(cos(theta_s), 2.0) + zb / 2.0 * sin(2.0 * theta_s) + gravity_;

    embedding_dynamics(3) = fmin(q_over, fmax(q_under, q)) + w(3);

    // ------------------------- D5 ----------------------------------------

    if (cos(embedding_state.get_x(4)) <= 0.) {
        y1 = embedding_state.get_x(0);
        y7 = embedding_state.get_xh(6);
    } else {
        y1 = embedding_state.get_xh(0);
        y7 = embedding_state.get_x(6);
    }

    if (sin(embedding_state.get_x(4)) <= 0.) {
        y3 = embedding_state.get_x(2);
        y8 = embedding_state.get_xh(7);
    } else {
        y3 = embedding_state.get_xh(2);
        y8 = embedding_state.get_x(7);
    }

    za = spring_constant_ * tanh(spring_saturation_ * (y8 - safe_displacement_z_)) - mass1_ * gravity_ -
         spring_dampener_ * y3;
    zb = spring_constant_ * tanh(spring_saturation_ * (y7 - safe_displacement_y_)) - spring_dampener_ * y1;
    double za_zb_norm = sqrt(pow(za, 2.0) + pow(zb, 2.0));

    theta_s = atan2(za, zb);
    theta_ss = -atan2(za, zb);

    J = k_roll_P_ * za / za_zb_norm * cos(theta_ss) - k_roll_P_ * zb / za_zb_norm * sin(theta_ss);

    q = k_roll_P_ * za / za_zb_norm * sin(embedding_state.get_x(4)) +
        k_roll_P_ * zb / za_zb_norm * cos(embedding_state.get_x(
                4)) + J * (embedding_state.get_x(4) - embedding_state.get_xh(4));
    q_over = k_roll_P_ * za / za_zb_norm * sin(theta_s) + k_roll_P_ * zb / za_zb_norm * cos(theta_s);
    q_under = -k_roll_P_ * za / za_zb_norm * sin(theta_s) - k_roll_P_ * zb / za_zb_norm * cos(theta_s);

    embedding_dynamics(4) = fmin(q_over, fmax(q_under, q)) + w(4);


    // ------------------------- D6 ----------------------------------------

    if (cos(embedding_state.get_x(5)) <= 0.) {
        y2 = embedding_state.get_x(1);
        y7 = embedding_state.get_x(6);
    } else {
        y2 = embedding_state.get_xh(1);
        y7 = embedding_state.get_xh(6);
    }

    if (sin(embedding_state.get_x(5)) <= 0.) {
        y4 = embedding_state.get_x(3);
        y8 = embedding_state.get_x(7);
    } else {
        y4 = embedding_state.get_xh(3);
        y8 = embedding_state.get_xh(7);
    }

    za = -spring_constant_ * tanh(spring_saturation_ * (y8 - safe_displacement_z_)) - mass1_ * gravity_ -
         spring_dampener_ * y4;
    zb = -spring_constant_ * tanh(spring_saturation_ * (y7 - safe_displacement_y_)) - spring_dampener_ * y2;

    za_zb_norm = sqrt(pow(za, 2.0) + pow(zb, 2.0));

    theta_s = atan2(za, zb);
    theta_ss = -atan2(za, zb);

    J = k_roll_P_ * za / za_zb_norm * cos(theta_ss) - k_roll_P_ * zb / za_zb_norm * sin(theta_ss);

    q = k_roll_P_ * za / za_zb_norm * sin(embedding_state.get_x(5)) +
        k_roll_P_ * zb / za_zb_norm * cos(embedding_state.get_x(
                5)) + J * (embedding_state.get_x(5) - embedding_state.get_xh(5));
    q_over = k_roll_P_ * za / za_zb_norm * sin(theta_s) + k_roll_P_ * zb / za_zb_norm * cos(theta_s);
    q_under = -k_roll_P_ * za / za_zb_norm * sin(theta_s) - k_roll_P_ * zb / za_zb_norm * cos(theta_s);

    embedding_dynamics(5) = fmin(q_over, fmax(q_under, q)) + w(5);

    // ------------------------- D7 and D8 ----------------------------------
    embedding_dynamics(6) = embedding_state.get_x(1) - embedding_state.get_xh(0);
    embedding_dynamics(7) = embedding_state.get_x(3) - embedding_state.get_xh(2);

    return embedding_dynamics;
}

void ASIF::backupTrajectoryDistribution(const int &traj_t, const int &backup_steps, std::queue<int> &out) const {
    int n_right = static_cast<int>(round(0.5 * (-pow((((double) traj_t - (double) backup_steps / 2.0)), corner_power_) /
                                                pow((double) backup_steps / 2.0, corner_power_) + 1) *
                                         (double) log_distribution_steps_) + 1);
    int n_left = log_distribution_steps_ - n_right + 1;

    auto log_right_step = static_cast<double>((exp(((double) backup_steps - (double) traj_t) * spacing_factor_) - 1) /
                                              ((double) n_right - 1));
    auto log_left_step = static_cast<double>((exp(((double) traj_t) * spacing_factor_) - 1) / ((double) n_left - 1));

    for (int i = 0; i < n_left; i++) {
        out.push(static_cast<int>(round(log(1 + log_left_step * (double) i) / spacing_factor_)));
    }
    for (int j = n_right - 1; j >= 0; j--) {
        out.push(static_cast<int>(backup_horizon_ - round(log(1 + log_right_step * (double) j) / spacing_factor_)));
    }
}
