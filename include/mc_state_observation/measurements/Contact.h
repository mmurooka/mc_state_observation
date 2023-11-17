#pragma once
#include <boost/assert.hpp>
#include <Eigen/Core>
#include <string>
#include <string_view>

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
  inline Contact(int id, std::string_view name) : id_(id), name_(name) {}
  // constructor if the contact is associated to a surface
  inline Contact(int id, std::string_view name, std::string_view surface) : Contact(id, name) { setSurface(surface); }
  inline bool operator<(const Contact & rhs) const noexcept { return (id() < rhs.id_); }

public:
  inline void resetContact() noexcept
  {
    wasAlreadySet_ = false;
    isSet_ = false;
  }

  // getters
  inline int id() const noexcept { return id_; }
  inline const std::string & name() const noexcept { return name_; }
  const std::string & surface() const
  {
    BOOST_ASSERT(!surface_.empty() && "The contact was created without a surface.");
    return surface_;
  }

  // getters
  void setSurface(std::string_view surfaceName) { surface_ = surfaceName; }

public:
  int id_;
  std::string name_;

  bool isSet_ = false;
  bool wasAlreadySet_ = false;

protected:
  std::string surface_;
};
} // namespace mc_state_observation::measurements
