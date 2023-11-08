#pragma once
#include <string>
namespace mc_state_observation::measurements
{

/**
 * Object making easier the handling of sensors within the observers.
 **/

/// @brief Class containing the information of a sensor to facilitate its handling.
struct Sensor
{

protected:
  inline Sensor() = default;
  inline Sensor(int id, std::string name) : id_(id), name_(name) {}

  inline bool operator<(const Sensor & contact2) const noexcept { return (id() < contact2.id_); }

public:
  inline const int & id() const noexcept { return id_; }
  inline const std::string & name() const noexcept { return name_; }

protected:
  int id_;
  std::string name_;
};
} // namespace mc_state_observation::measurements
