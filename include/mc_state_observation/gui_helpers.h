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

auto make_motionvecd_input(std::string name, sva::MotionVecd & vel)
{
  return ArrayInputImpl(name,
                    {"wx [rad/s]", "wy [rad/s]", "wz [rad/s]", "vx [m/s]", "vy [m/s]", "vz [m/s]"},
                  [&vel]() -> const sva::MotionVecd &
                  {
                   return vel;
                  },
                  [&vel](const sva::MotionVecd & newVel)
                  {
                    vel = newVel;
                  });
}

auto make_admittancevecd_input(std::string name, sva::AdmittanceVecd & vel)
{
  return ArrayInputImpl(name,
                    {"wx", "wy", "wz", "vx", "vy", "vz"},
                  [&vel]() -> Eigen::Vector6d
                  {
                   return vel.vector();
                  },
                  [&vel](const Eigen::Vector6d & newVel)
                  {
                    vel = newVel;
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

auto make_label(std::string name, const std::string & label)
{
  return Label(name,
    [label]()
    {
      return label;
    });
}

} /* gui */

} /*  mc_rtc
 */

