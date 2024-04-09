#include "kalman_filter.hpp"

KalmanFilter::~KalmanFilter() {}

KalmanFilter::KalmanFilter(ClockGenerator* clock_generator, FfComponents& components): Component(1,clock_generator), components_(components) {
    this->covariance = libra::Matrix<3,3>(100.0) + 10.0 * libra::MakeIdentityMatrix<3>();
    this->estimated_position = libra::Vector<3>(0);
    this->estimated_position[1] = 10;
    this->obs_position = libra::MakeIdentityMatrix<3>();
    this->obs_distance = libra::Matrix<1,3>(0);
    this->obs_distance[0][1] = 1.0;
}

void KalmanFilter::MainRoutine(const int time_count) {

    if (time_count < 2){
        return;
    }

    const libra::Vector<3> relative_position = components_.GetRelativePositionSensor().GetMeasuredTargetPosition_rtn_m();
    const libra::Vector<3> relative_velocity = components_.GetRelativeVelocitySensor().GetMeasuredTargetVelocity_rtn_m_s();
    const double relative_distance = components_.GetRelativeDistanceSensor().GetMeasuredDistance_m();

    this->estimated_position += 0.1 * relative_velocity;
    this->covariance += 0.2 * libra::MakeIdentityMatrix<3>();

    if (time_count % 8 == 7) {
        try
        {
            auto gain = covariance * obs_position.Transpose() * libra::CalcInverseMatrix(0.1 * libra::MakeIdentityMatrix<3>() + obs_position * covariance * obs_position.Transpose());
            auto ratio = libra::MakeIdentityMatrix<3>() - gain * obs_position;
            this->covariance = ratio * covariance * ratio.Transpose() + 0.1 * gain * gain.Transpose();

            this->estimated_position += gain * (relative_position - obs_position * estimated_position);
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

    if (time_count % 3 == 2) {
        try
        {
            obs_distance[0][0] = estimated_position[0] / estimated_position.CalcNorm();
            obs_distance[0][1] = estimated_position[1] / estimated_position.CalcNorm();
            obs_distance[0][2] = estimated_position[2] / estimated_position.CalcNorm();
            auto gain = covariance * obs_distance.Transpose() * libra::CalcInverseMatrix(libra::MakeIdentityMatrix<1>() + obs_distance * covariance * obs_distance.Transpose());
            auto ratio = libra::MakeIdentityMatrix<3>() - gain * obs_distance;
            this->covariance = ratio * covariance * ratio.Transpose() + gain * gain.Transpose();

            this->estimated_position += gain * (libra::Vector<1>(relative_distance) - obs_distance * estimated_position);
        }
        catch(const std::invalid_argument& e)
        {
            std::cout << "test" << std::endl;
            std::cout << estimated_position.CalcNorm() << std::endl;
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
  str_tmp += WriteVector(head + "estimated_position", "rtn", "m", 3);
  str_tmp += WriteVector(head + "covariance_position", "rtn", "m", 3);
  return str_tmp;
}

std::string KalmanFilter::GetLogValue() const {
  std::string str_tmp = "";
  libra::Vector<3> cov_vector;

  cov_vector[0] = covariance[0][0];
  cov_vector[1] = covariance[1][1];
  cov_vector[2] = covariance[2][2];
  
  str_tmp += WriteVector(estimated_position);
  str_tmp += WriteVector(cov_vector);

  return str_tmp;
}
