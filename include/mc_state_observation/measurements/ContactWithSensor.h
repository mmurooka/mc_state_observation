#pragma once

#include <mc_state_observation/measurements/Contact.h>

namespace mc_state_observation::measurements
{
/**
 * Object that contains all the functions and necessary information making easier the handling of contacts associated to
 * a sensor within the observers.

 * If the contact is detected using a thresholding on the contact force, the contact force cannot be obtained and the
 * name of the contact will be the one of the force sensor. Otherwise the name of the contact surface is used, allowing
 * the creation of contacts associated to a same sensor but a different surface.
 **/
struct ContactWithSensor : public Contact
{

public:
  inline ContactWithSensor() = default;
  // constructor if the contact is not associated to a surface
  // its name will be the name of the force sensor
  ContactWithSensor(int id, std::string_view forceSensorName)
  : Contact(id, forceSensorName), forceSensor_(forceSensorName)
  {
  }

  // constructor if the contact is associated to a surface
  // its name will be the name of the force sensor
  ContactWithSensor(int id, std::string_view forceSensorName, std::string_view surfaceName)
  : Contact(id, forceSensorName, surfaceName), forceSensor_(forceSensorName)
  {
  }

  inline void resetContact() noexcept
  {
    Contact::resetContact();
    sensorWasEnabled_ = false;
  }

  const std::string & forceSensor() const noexcept { return forceSensor_; }

public:
  Eigen::Matrix<double, 6, 1> wrenchInCentroid_ = Eigen::Matrix<double, 6, 1>::Zero(); // for logs
  double forceNorm_ = 0.0;
  // the sensor measurement has to be used by the observer
  bool sensorEnabled_ = true;
  // allows to know if the contact's measurements have to be added during the update.
  bool sensorWasEnabled_ = false;

  // measured contact wrench, expressed in the frame of the contact. Not automatically computed so must be explicitely
  // computed and called.
  Eigen::Matrix<double, 6, 1> contactWrenchVector_;

protected:
  std::string forceSensor_;
};
} // namespace mc_state_observation::measurements
