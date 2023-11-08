#pragma once
#include <boost/assert.hpp>
#include <Eigen/Core>
#include <string>

namespace mc_state_observation::measurements
{
/**
 * Object making easier the handling of contacts within the observers.
 **/

struct Contact
{
protected:
  inline Contact() = default;
  // constructor if the contact is not associated to a surface
  inline Contact(int id, std::string name)
  {
    id_ = id;
    name_ = name;
  }
  // constructor if the contact is associated to a surface
  inline Contact(int id, std::string name, std::string surface) : Contact(id, name) { setSurfaceName(surface); }
  inline bool operator<(const Contact & contact2) const noexcept { return (id() < contact2.id_); }

public:
  inline void resetContact()
  {
    wasAlreadySet_ = false;
    isSet_ = false;
  }

  // getters
  inline const int & id() const noexcept { return id_; }
  inline const std::string & name() const noexcept { return name_; }
  const std::string & surfaceName() const
  {
    BOOST_ASSERT(!surface_.empty() && "The contact was created without a surface.");
    return surface_;
  }

  // getters
  void setSurfaceName(std::string surfaceName) { surface_ = surfaceName; }

  /*// ! Not working yet
  inline const Eigen::Vector3d & getZMP()
  {
    return zmp;
  }
  */

public:
  int id_;
  std::string name_;

  bool isSet_ = false;
  bool wasAlreadySet_ = false;
  // Eigen::Vector3d zmp; // ! Not working yet
protected:
  std::string surface_;
};
} // namespace mc_state_observation::measurements