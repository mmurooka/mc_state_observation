#include <mc_rtc/gui.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/constants.h>

namespace  mc_rtc
{
namespace gui
{

auto make_checkbox(std::string name, bool & variable)
{
  return CheckboxImpl(name,
                  [&variable]()
                  {
                   return variable;
                  },
                  [&variable]()
                  {
                   variable = !variable;
                  });
}

template<typename T>
auto make_number_input(std::string name, T & value)
{
  return NumberInputImpl(name,
                  [&value]()
                  {
                   return value;
                  },
                  [&value](const T & newValue)
                  {
                   value = newValue;
                  });
}

auto make_rpy_input(std::string name, Eigen::Matrix3d & orientation)
{
  return ArrayInputImpl(name,
                    {"r [deg]", "p [deg]", "y [deg]"},
                  [&orientation]() -> Eigen::Vector3d
                  {
                   return mc_rbdyn::rpyFromMat(orientation) * 180./mc_rtc::constants::PI;
                  },
                  [&orientation](const Eigen::Vector3d & rpy)
                  {
                   orientation = mc_rbdyn::rpyToMat(rpy * mc_rtc::constants::PI/180);
                  });
}

auto make_rpy_label(std::string name, const Eigen::Matrix3d & orientation)
{
  return ArrayLabel(name,
                    {"r [deg]", "p [deg]", "y [deg]"},
                  [&orientation]() -> Eigen::Vector3d
                  {
                   return mc_rbdyn::rpyFromMat(orientation) * 180./mc_rtc::constants::PI;
                  });
}

} /* gui */

} /*  mc_rtc
 */

