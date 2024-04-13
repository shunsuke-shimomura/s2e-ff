#include "relative_acceleration_sensor.hpp"

#include <components/base/sensor.hpp>
#include <library/initialize/initialize_file_access.hpp>

RelativeAccelerationSensor::RelativeAccelerationSensor(const int prescaler, ClockGenerator* clock_gen, Sensor& sensor_base, const int target_sat_id, const int reference_sat_id, const RelativeInformation& rel_info, const Dynamics& dynamics, const double compo_step_time_s)
    : Component(prescaler, clock_gen),
    Sensor(sensor_base),
    target_sat_id_(target_sat_id),
    reference_sat_id_(reference_sat_id),
    rel_info_(rel_info),
    dynamics_(dynamics),
    compo_step_time_s_(compo_step_time_s)
    {}

RelativeAccelerationSensor::~RelativeAccelerationSensor() {}

void RelativeAccelerationSensor::MainRoutine(int count) {
    UNUSED(count);

    libra::Quaternion q_i2rtn = dynamics_.GetOrbit().CalcQuaternion_i2lvlh();

    libra::Vector<3> true_target_velocity_rtn_m_s = rel_info_.GetRelativeVelocity_rtn_m_s(target_sat_id_, reference_sat_id_);

    libra::Vector<3> true_target_acceleration_rtn_m_s2 = (1.0 / compo_step_time_s_) * (true_target_velocity_rtn_m_s - last_target_velocity_rtn_m_s_);

    measured_target_acceleration_rtn_m_s2_ = Measure(true_target_acceleration_rtn_m_s2);

    measured_target_acceleration_i_m_s2_ = q_i2rtn.InverseFrameConversion(true_target_acceleration_rtn_m_s2);

    last_target_velocity_rtn_m_s_ = true_target_velocity_rtn_m_s;
}

std::string RelativeAccelerationSensor::GetLogHeader() const {
    std::string str_tmp = "";
    std::string head = "RelativeAccelerationSensor_";
    str_tmp += WriteVector(head + "acceleration", "i", "m/s2", 3);
    str_tmp += WriteVector(head + "acceleration", "rtn", "m/s2",3);
    return str_tmp;
}

std::string RelativeAccelerationSensor::GetLogValue() const {
    std::string str_tmp = "";

    str_tmp += WriteVector(measured_target_acceleration_i_m_s2_);
    str_tmp += WriteVector(measured_target_acceleration_rtn_m_s2_);

    return str_tmp;
}

RelativeAccelerationSensor InitializeRelativeAccelerationSensor(ClockGenerator* clock_gen, const std::string file_name, const double compo_step_time_s, const RelativeInformation& rel_info, const Dynamics& dynamics, const int reference_sat_id_input) {
  IniAccess ini_file(file_name);
  char section[30] = "RELATIVE_ACCELERATION_SENSOR";

  int prescaler = ini_file.ReadInt(section, "prescaler");
  if (prescaler <= 1) prescaler = 1;

  int target_sat_id = ini_file.ReadInt(section,"target_sat_id");
  int reference_sat_id = ini_file.ReadInt(section, "reference_sat_id");
  if (reference_sat_id < 0) {
    reference_sat_id = reference_sat_id_input;
  }
  Sensor<3> sensor_base = ReadSensorInformation<3>(file_name, compo_step_time_s * (double)(prescaler), section, "m_s2");

  RelativeAccelerationSensor relative_acceleration_sensor(prescaler, clock_gen, sensor_base, target_sat_id, reference_sat_id, rel_info, dynamics, compo_step_time_s);

  return relative_acceleration_sensor;
}