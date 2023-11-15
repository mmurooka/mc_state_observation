#include <string>
#include <variant>
#include <vector>

namespace mc_state_observation::measurements
{
namespace internal
{
template<class ConfigurationType>
struct ContactsManagerConfigurationPrvt
{
public:
  ContactsManagerConfigurationPrvt(const std::string & observerName) : observerName_(observerName) {}

  ConfigurationType & contactDetectionThreshold(double contactDetectionThreshold)
  {
    contactDetectionThreshold_ = contactDetectionThreshold;
    return static_cast<ConfigurationType &>(*this);
  }
  ConfigurationType & verbose(bool verbose)
  {
    verbose_ = verbose;
    return static_cast<ConfigurationType &>(*this);
  }

public:
  std::string observerName_;

  double contactDetectionThreshold_ = 0.11;
  bool verbose_ = true;
};
} // namespace internal

struct ContactsManagerSurfacesConfiguration
: public internal::ContactsManagerConfigurationPrvt<ContactsManagerSurfacesConfiguration>
{
public:
  ContactsManagerSurfacesConfiguration(const std::string & observerName,
                                       const std::vector<std::string> & surfacesForContactDetection)
  : ContactsManagerConfigurationPrvt<ContactsManagerSurfacesConfiguration>(observerName),
    surfacesForContactDetection_(surfacesForContactDetection)
  {
  }

  ContactsManagerSurfacesConfiguration & contactSensorsDisabledInit(
      const std::vector<std::string> & contactSensorsDisabledsInit)
  {
    contactSensorsDisabledInit_ = contactSensorsDisabledsInit;
    return *this;
  }

  // list of admissible contact surfaces for the detection
  std::vector<std::string> surfacesForContactDetection_;
  // list of sensors that must not be used from the start of the observer
  std::vector<std::string> contactSensorsDisabledInit_ = std::vector<std::string>();
};

struct ContactsManagerSensorsConfiguration
: public internal::ContactsManagerConfigurationPrvt<ContactsManagerSensorsConfiguration>
{

  ContactsManagerSensorsConfiguration(const std::string & observerName)
  : ContactsManagerConfigurationPrvt<ContactsManagerSensorsConfiguration>(observerName)
  {
  }

  ContactsManagerSensorsConfiguration & forceSensorsToOmit(const std::vector<std::string> & forceSensorsToOmit)
  {
    forceSensorsToOmit_ = forceSensorsToOmit;
    return *this;
  }
  ContactsManagerSensorsConfiguration & contactSensorsDisabledInit(
      const std::vector<std::string> & contactSensorsDisabledsInit)
  {
    contactSensorsDisabledInit_ = contactSensorsDisabledsInit;
    return *this;
  }

public:
  // force sensors that must not be used for the contacts detection (ex: hands when holding an object)
  std::vector<std::string> forceSensorsToOmit_;
  // list of sensors that must not be used from the start of the observer
  std::vector<std::string> contactSensorsDisabledInit_ = std::vector<std::string>();
};

struct ContactsManagerSolverConfiguration
: public internal::ContactsManagerConfigurationPrvt<ContactsManagerSolverConfiguration>
{
public:
  ContactsManagerSolverConfiguration(const std::string & observerName)
  : ContactsManagerConfigurationPrvt<ContactsManagerSolverConfiguration>(observerName)
  {
  }
};

} // namespace mc_state_observation::measurements
