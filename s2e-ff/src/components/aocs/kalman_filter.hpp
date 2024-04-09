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
public:
    KalmanFilter(ClockGenerator* clock_generator, FfComponents& components);
    ~KalmanFilter();

    FfComponents& components_;
    libra::Vector<3> estimated_position;
    libra::Matrix<3,3> covariance;
    libra::Matrix<3,3> obs_position;
    libra::Matrix<1,3> obs_distance;

    void MainRoutine(const int time_count);

    virtual std::string GetLogHeader() const;

    virtual std::string GetLogValue() const;
};

#endif