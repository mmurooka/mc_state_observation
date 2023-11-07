#pragma once
#include <boost/assert.hpp>
#include <Eigen/Core>
#include <mc_state_observation/measurements/Sensor.h>
#include <string>

namespace mc_state_observation::measurements
{
/**
 * Object making easier the handling of contacts within the observers.
 **/

struct Contact : public Sensor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  Contact() {}
  ~Contact() {}
  // constructor if the contact is not associated to a surface
  Contact(int id, std::string name)
  {
    id_ = id;
    name_ = name;
    resetContact();
  }
  // constructor if the contact is associated to a surface
  Contact(int id, std::string name, std::string surface) : Contact(id, name) { surfaceName(surface); }

public:
  inline void resetContact()
  {
    wasAlreadySet_ = false;
    isSet_ = false;
  }

  void surfaceName(std::string surfaceName) { surface_ = surfaceName; }
  const std::string & surfaceName() const
  {
    BOOST_ASSERT(!surface_.empty() && "The contact was created without a surface.");
    return surface_;
  }

  /*// ! Not working yet
  inline const Eigen::Vector3d & getZMP()
  {
    return zmp;
  }
  */

public:
  bool isSet_ = false;
  bool wasAlreadySet_ = false;
  // Eigen::Vector3d zmp; // ! Not working yet
protected:
  std::string surface_;
};
} // namespace mc_state_observation::measurements