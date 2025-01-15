#include "kalman_filter.hpp"

KalmanFilter::~KalmanFilter() {}

KalmanFilter::KalmanFilter(ClockGenerator* clock_generator, FfComponents& components, const double acc_std, const double vel_std, const double pos_std, const double dist_std, const int vel_step, const int pos_step, const int dist_step, const libra::Vector<6>& init_state, const double step_time_s): Component(1,clock_generator), components_(components), pos_std_(pos_std), vel_std_(vel_std), acc_std_(acc_std), dist_std_(dist_std), pos_step_(pos_step), vel_step_(vel_step), dist_step_(dist_step), step_time_s_(step_time_s) {
    this->covariance = libra::Matrix<6,6>(100.0) + 10.0 * libra::MakeIdentityMatrix<6>();
    this->estimated_state = init_state;
    this->state_transition = libra::Matrix<6,6>(0);
    this->state_transition[0][0] = 1.0;
    this->state_transition[1][1] = 1.0;
    this->state_transition[2][2] = 1.0;
    this->state_transition[0][3] = step_time_s_;
    this->state_transition[1][4] = step_time_s_;
    this->state_transition[2][5] = step_time_s_;
    this->state_transition[3][3] = 1.0;
    this->state_transition[4][4] = 1.0;
    this->state_transition[5][5] = 1.0;
    this->control_input = libra::Matrix<6,3>(0);
    this->control_input[0][0] = step_time_s_*step_time_s_/2.0;
    this->control_input[1][1] = step_time_s_*step_time_s_/2.0;
    this->control_input[2][2] = step_time_s_*step_time_s_/2.0;
    this->control_input[3][0] = step_time_s_;
    this->control_input[4][1] = step_time_s_;
    this->control_input[5][2] = step_time_s_;
    this->obs_position = libra::Matrix<3,6>(0);
    this->obs_position[0][0] = 1.0;
    this->obs_position[1][1] = 1.0;
    this->obs_position[2][2] = 1.0;
    this->obs_velocity = libra::Matrix<3,6>(0);
    this->obs_velocity[0][3] = 1.0;
    this->obs_velocity[1][4] = 1.0;
    this->obs_velocity[2][5] = 1.0;
    this->obs_distance = libra::Matrix<1,6>(0);
    this->obs_distance[0][1] = 1.0;
}

void KalmanFilter::MainRoutine(const int time_count) {

    if (time_count < 3){
        return;
    }

    const libra::Vector<3> relative_acceleration = components_.GetRelativeAccelerationSensor().GetMeasuredTargetAcceleration_rtn_m_s2();
    const libra::Vector<3> relative_position = components_.GetRelativePositionSensor().GetMeasuredTargetPosition_rtn_m();
    const libra::Vector<3> relative_velocity = components_.GetRelativeVelocitySensor().GetMeasuredTargetVelocity_rtn_m_s();
    const double relative_distance = components_.GetRelativeDistanceSensor().GetMeasuredDistance_m();

    this->estimated_state = this->state_transition * this->estimated_state + this->control_input * relative_acceleration;
    this->covariance = this->state_transition * this->covariance * this->state_transition.Transpose() + this->control_input * (this->acc_std_ * this->acc_std_ * libra::MakeIdentityMatrix<3>()) * this->control_input.Transpose();

    if (time_count % this->pos_step_ == 0) {
        try
        {
            auto gain = covariance * obs_position.Transpose() * libra::CalcInverseMatrix(this->pos_std_ * this->pos_std_ * libra::MakeIdentityMatrix<3>() + obs_position * covariance * obs_position.Transpose());
            auto ratio = libra::MakeIdentityMatrix<6>() - gain * obs_position;
            this->covariance = ratio * covariance * ratio.Transpose() + gain * (this->pos_std_ * this->pos_std_ * libra::MakeIdentityMatrix<3>()) * gain.Transpose();

            auto tmp = gain * (relative_position - obs_position * estimated_state);
            auto est = this->estimated_state + gain * (relative_position - obs_position * estimated_state);
            this->estimated_state = est;
        }
        catch(const std::invalid_argument& e)
        {
            std::cout << "test" << std::endl;
            std::cout << time_count << std::endl;
            (0.1 * libra::MakeIdentityMatrix<3>() + obs_position * covariance * obs_position.Transpose()).Print();
            std::cout << "position" << std::endl;
            throw e;
        } 
    }

    if (time_count % vel_step_ == 0) {
        try
        {
            auto gain = covariance * obs_velocity.Transpose() * libra::CalcInverseMatrix(this->vel_std_ * this->vel_std_ * libra::MakeIdentityMatrix<3>() + obs_velocity * covariance * obs_velocity.Transpose());
            auto ratio = libra::MakeIdentityMatrix<6>() - gain * obs_velocity;
            this->covariance = ratio * covariance * ratio.Transpose() + gain * (this->vel_std_ * this->vel_std_ * libra::MakeIdentityMatrix<3>()) * gain.Transpose();

            this->estimated_state += gain * (libra::Vector<3>(relative_velocity) - obs_velocity * estimated_state);
        }
        catch(const std::invalid_argument& e)
        {
            std::cout << "test" << std::endl;
            std::cout << estimated_state.CalcNorm() << std::endl;
            std::cout << time_count << std::endl;
            (libra::MakeIdentityMatrix<1>() + obs_distance * covariance * obs_distance.Transpose()).Print();
            std::cout << "distance" << std::endl;
            throw e;
        }       
    }

    if (time_count % dist_step_ == 0 && false) {
        try
        {
            obs_distance[0][0] = estimated_state[0] / estimated_state.CalcNorm();
            obs_distance[0][1] = estimated_state[1] / estimated_state.CalcNorm();
            obs_distance[0][2] = estimated_state[2] / estimated_state.CalcNorm();
            auto gain = covariance * obs_distance.Transpose() * libra::CalcInverseMatrix(this->vel_std_ * this->vel_std_ * libra::MakeIdentityMatrix<1>() + obs_distance * covariance * obs_distance.Transpose());
            auto ratio = libra::MakeIdentityMatrix<6>() - gain * obs_distance;
            this->covariance = ratio * covariance * ratio.Transpose() + gain * (this->dist_std_ * this->dist_std_ * libra::MakeIdentityMatrix<1>()) * gain.Transpose();

            this->estimated_state += gain * (libra::Vector<1>(relative_distance) - obs_distance * estimated_state);
        }
        catch(const std::invalid_argument& e)
        {
            std::cout << "test" << std::endl;
            std::cout << estimated_state.CalcNorm() << std::endl;
            std::cout << time_count << std::endl;
            (libra::MakeIdentityMatrix<1>() + obs_distance * covariance * obs_distance.Transpose()).Print();
            std::cout << "distance" << std::endl;
            throw e;
        }       
    }
}

std::string KalmanFilter::GetLogHeader() const {
  std::string str_tmp = "";
  std::string head = "kalman_filter_";
  str_tmp += WriteVector(head + "estimated_state", "rtn", "m", 6);
  str_tmp += WriteVector(head + "covariance_state", "rtn", "m", 6);
  return str_tmp;
}

std::string KalmanFilter::GetLogValue() const {
  std::string str_tmp = "";
  libra::Vector<6> cov_vector;

  cov_vector[0] = covariance[0][0];
  cov_vector[1] = covariance[1][1];
  cov_vector[2] = covariance[2][2];
  cov_vector[3] = covariance[3][3];
  cov_vector[4] = covariance[4][4];
  cov_vector[5] = covariance[5][5];
  
  str_tmp += WriteVector(estimated_state);
  str_tmp += WriteVector(cov_vector);

  return str_tmp;
}

KalmanFilter InitializeKalmanFilter(ClockGenerator* clock_gen, const std::string file_name, FfComponents& components, const double step_time_s) {
    IniAccess ini_file(file_name);
    char section[30] = "KALMAN_FILTER";

    double acc_std = ini_file.ReadDouble(section,"acceleration_std");
    double vel_std = ini_file.ReadDouble(section,"velocity_std");
    double pos_std = ini_file.ReadDouble(section,"position_std");
    double dist_std = ini_file.ReadDouble(section,"distance_std");
    
    int vel_step = ini_file.ReadInt(section, "velocity_step");
    int pos_step = ini_file.ReadInt(section, "position_step");
    int dist_step = ini_file.ReadInt(section,"distance_step");

    libra::Vector<6> init_state;
    ini_file.ReadVector(section, "initial_state", init_state);

    KalmanFilter kf(clock_gen, components, acc_std, vel_std, pos_std, dist_std, vel_step, pos_step, dist_step, init_state, step_time_s);

    return kf;
}
