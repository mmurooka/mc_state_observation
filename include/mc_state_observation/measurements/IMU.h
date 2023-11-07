#pragma once
#include <Eigen/Core>
#include <mc_state_observation/measurements/Sensor.h>
#include <string>

namespace mc_state_observation::measurements
{
/**
 * Object making easier the handling of sensors within the observers.
 **/

/// @brief Class containing the information of an IMU.
struct IMU : public Sensor
{
public:
  IMU(int id, std::string name)
  {
    id_ = id;
    name_ = name;
    gyroBias << 0.0, 0.0, 0.0;
  }
  ~IMU() {}

public:
  Eigen::Vector3d gyroBias;
};
} // namespace mc_state_observation::measurements