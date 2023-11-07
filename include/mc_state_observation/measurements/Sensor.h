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
public:
  inline const int & getID() const { return id_; }
  inline const std::string & getName() const { return name_; }

protected:
  Sensor() {}
  ~Sensor() {}
  Sensor(int id, std::string name) : id_(id), name_(name) {}

  bool operator<(const Sensor & contact2) const { return (getID() < contact2.id_); }

protected:
  int id_;
  std::string name_;
};
} // namespace mc_state_observation::measurements