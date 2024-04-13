#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <components/base/component.hpp>
#include <library/math/vector.hpp>
#include "../../simulation/spacecraft/ff_components.hpp"

class FfComponents;

class KalmanFilter: public Component, public ILoggable
{
private:
    /* data */
    double acc_std_;
    double vel_std_;
    double pos_std_;
    double dist_std_;

    int vel_step_;
    int pos_step_;
    int dist_step_;

    double step_time_s_;
public:
    KalmanFilter(ClockGenerator* clock_generator, FfComponents& components, const double acc_std, const double vel_std, const double pos_std, const double dist_std, const int vel_step, const int pos_step, const int dist_step, const libra::Vector<6>& init_state, const double step_time_s);
    ~KalmanFilter();

    FfComponents& components_;
    libra::Vector<6> estimated_state;
    libra::Matrix<6,6> covariance;
    libra::Matrix<6,6> state_transition;
    libra::Matrix<6,3> control_input;
    libra::Matrix<3,6> obs_position;
    libra::Matrix<3,6> obs_velocity;
    libra::Matrix<1,6> obs_distance;

    void MainRoutine(const int time_count);

    virtual std::string GetLogHeader() const;

    virtual std::string GetLogValue() const;
};

KalmanFilter InitializeKalmanFilter(ClockGenerator* clock_gen, const std::string file_name, FfComponents& components, const double step_time_s);

#endif