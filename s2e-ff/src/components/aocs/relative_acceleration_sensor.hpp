#ifndef S2E_COMPONENTS_AOCS_RELATIVE_ACCELERATION_SENSOR_HPP_
#define S2E_COMPONENTS_AOCS_RELATIVE_ACCELERATION_SENSOR_HPP_

#include <components/base/component.hpp>
#include <components/base/sensor.hpp>
#include <library/logger/logger.hpp>
#include <simulation/multiple_spacecraft/relative_information.hpp>

class RelativeAccelerationSensor : public Component, public Sensor<3>, public ILoggable {
    public:
    RelativeAccelerationSensor(const int prescaler, ClockGenerator* clock_gen, Sensor& sensor_base, const int target_sat_id, const int reference_sat_id, const RelativeInformation& rel_info, const Dynamics& dynamics);

    ~RelativeAccelerationSensor();

    void MainRoutine(int const);

    virtual std::string GetLogHeader() const;

    virtual std::string GetLogValue() const;

    inline libra::Vector<3> GetMeasuredTargetAcceleration_rtn_m_s() const {return measured_target_acceleration_rtn_m_s2_;}

    protected:
    int target_sat_id_;

    const int reference_sat_id_;

    libra::Vector<3> measured_target_acceleration_rtn_m_s2_{0.0};
    libra::Vector<3> measured_target_acceleration_i_m_s2_{0.0};
    libra::Vector<3> last_target_velocity_rtn_m_s_{0.0};

    const RelativeInformation& rel_info_;
    const Dynamics& dynamics_;
};

RelativeAccelerationSensor InitializeRelativeAccelerationSensor (ClockGenerator* clock_gen, const std::string file_name, const double compo_step_time_s, const RelativeInformation& rel_info, const Dynamics& dynamics, const int reference_sat_id_input = -1);

#endif // S2E_COMPONENTS_AOCS_RELATIVE_ACCELERATION_SENSOR_HPP_