#pragma once
#include "mc_state_observation/measurements/measurementsTools.h"

namespace mc_state_observation::measurements
{

///////////////////////////////////////////////////////////////////////
/// ------------------------------Contacts-----------------------------
///////////////////////////////////////////////////////////////////////

template<typename ContactWithSensorT>
void ContactsManager<ContactWithSensorT>::init(const std::string & observerName, const bool verbose)
{
  observerName_ = observerName;
  verbose_ = verbose;
}

template<typename ContactWithSensorT>
void ContactsManager<ContactWithSensorT>::initDetection(const mc_control::MCController & ctl,
                                                        const std::string & robotName,
                                                        ContactsDetection contactsDetection,
                                                        const std::vector<std::string> & surfacesForContactDetection,
                                                        const std::vector<std::string> & contactsSensorDisabledInit,
                                                        const double & contactDetectionThreshold)
{

  contactsFinder_ = &ContactsManager<ContactWithSensorT>::findContactsFromSurfaces;

  contactDetectionThreshold_ = contactDetectionThreshold;
  surfacesForContactDetection_ = surfacesForContactDetection;
  contactsSensorDisabledInit_ = contactsSensorDisabledInit;

  const auto & robot = ctl.robot(robotName);

  if(contactsDetection != Solver && contactsDetection != Sensors && contactsDetection != Surfaces)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Contacts detection type not allowed. Please pick among : [Solver, Sensors, Surfaces] or "
        "initialize a list of surfaces with the variable surfacesForContactDetection");
  }

  if(surfacesForContactDetection.size() > 0)
  {
    if(contactsDetection != Surfaces)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "The list of potential contact surfaces was given but the detection using surfaces is not selected");
    }
  }
  else if(contactsDetection == Surfaces)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "You selected the contacts detection using surfaces but didn't add the list of surfaces, please add it using "
        "the variable surfacesForContactDetection");
  }

  if(contactsDetection == Surfaces)
  {
    for(const std::string & surface : surfacesForContactDetection)
    {
      // if the surface is associated to a force sensor (for example LeftFootCenter or RightFootCenter)
      if(robot.surfaceHasForceSensor(surface))
      {
        const mc_rbdyn::ForceSensor & forceSensor = robot.surfaceForceSensor(surface);
        const std::string & fsName = forceSensor.name();
        mapContacts_.insertContact(fsName, surface);
        addContactToGui(ctl, surface);
      }
      else // if the surface is not associated to a force sensor, we will fetch the force sensor indirectly attached to
           // the surface
      {
        const mc_rbdyn::ForceSensor & forceSensor = robot.indirectSurfaceForceSensor(surface);
        const std::string & fsName = forceSensor.name();
        mapContacts_.insertContact(fsName, surface);
        addContactToGui(ctl, surface);
      }
    }
  }

  for(auto const & contactSensorDisabledInit : contactsSensorDisabledInit)
  {
    BOOST_ASSERT(mapContacts_.hasElement(contactSensorDisabledInit) && "This sensor is not attached to the robot");
    mapContacts_.contact(contactSensorDisabledInit).sensorEnabled_ = false;
  }
}

template<typename ContactWithSensorT>
void ContactsManager<ContactWithSensorT>::initDetection(const mc_control::MCController & ctl,
                                                        const std::string & robotName,
                                                        ContactsDetection contactsDetection,
                                                        const std::vector<std::string> & contactsSensorDisabledInit,
                                                        const double & contactDetectionThreshold,
                                                        const std::vector<std::string> & forceSensorsToOmit)
{
  if(contactsDetection == Solver) { contactsFinder_ = &ContactsManager<ContactWithSensorT>::findContactsFromSolver; }
  if(contactsDetection == Sensors) { contactsFinder_ = &ContactsManager<ContactWithSensorT>::findContactsFromSensors; }

  contactDetectionThreshold_ = contactDetectionThreshold;
  contactsSensorDisabledInit_ = contactsSensorDisabledInit;

  const auto & robot = ctl.robot(robotName);

  if(contactsDetection != Solver && contactsDetection != Sensors && contactsDetection != Surfaces)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Contacts detection type not allowed. Please pick among : [Solver, Sensors, Surfaces] or "
        "initialize a list of surfaces with the variable surfacesForContactDetection");
  }

  if(contactsDetection == Surfaces)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "You selected the contacts detection using surfaces but didn't add the list of surfaces, please use the "
        "ContactsManager constructor that receives this surfaces list");
  }

  if(contactsDetection == Sensors)
  {
    for(auto forceSensor : robot.forceSensors())
    {
      if(std::find(forceSensorsToOmit.begin(), forceSensorsToOmit.end(), forceSensor.name())
         != forceSensorsToOmit.end())
      { continue; }
      const std::string & fsName = forceSensor.name();

      mapContacts_.insertContact(fsName);
      addContactToGui(ctl, fsName);
    }
  }

  for(auto const & contactSensorDisabledInit : contactsSensorDisabledInit)
  {
    BOOST_ASSERT(mapContacts_.hasElement(contactSensorDisabledInit) && "This sensor is not attached to the robot");
    mapContacts_.contact(contactSensorDisabledInit).sensorEnabled_ = false;
  }
}

template<typename ContactWithSensorT>
void ContactsManager<ContactWithSensorT>::addContactToGui(const mc_control::MCController & ctl,
                                                          const std::string & name)
{
  ctl.gui()->addElement({observerName_, "Contacts"},
                        mc_rtc::gui::Checkbox(
                            name + " : " + (mapContacts_.contact(name).isSet_ ? "Contact is set" : "Contact is not set")
                                + ": Use wrench sensor: ",
                            [this, name]() { return mapContacts_.contact(name).sensorEnabled_; },
                            [this, name]() {
                              mapContacts_.contact(name).sensorEnabled_ = !mapContacts_.contact(name).sensorEnabled_;
                              std::cout << std::endl
                                        << "Enable / disable :" + name + " "
                                               + std::to_string(mapContacts_.contact(name).sensorEnabled_)
                                        << std::endl;
                            }));
}

template<typename ContactWithSensorT>
const std::set<int> & ContactsManager<ContactWithSensorT>::findContacts(const mc_control::MCController & ctl,
                                                                        const std::string & robotName)
{
  // Detection of the contacts depending on the configured mode
  (this->*contactsFinder_)(ctl, robotName);
  updateContacts();

  return contactsFound_; // list of currently set contacts
}

template<typename ContactWithSensorT>
void ContactsManager<ContactWithSensorT>::findContactsFromSolver(const mc_control::MCController & ctl,
                                                                 const std::string & robotName)
{
  mc_rtc::log::warning("This mode has not been tested deeply, there might be issues with the contacts surfaces and "
                       "names. There seems to be an issue when the robot turns in LipmWalking using legged odometry. "
                       "This issue doesn't occur with the other detection methods so there must be a problem with the "
                       "contacts list or the contacts kinematics not turning? To check");
  const auto & measRobot = ctl.robot(robotName);

  contactsFound_.clear();
  for(const auto & contact : ctl.solver().contacts())
  {
    if(ctl.robots().robot(contact.r1Index()).name() == measRobot.name())
    {
      if(ctl.robots().robot(contact.r2Index()).mb().joint(0).type() == rbd::Joint::Fixed)
      {
        const std::string & surfaceName = contact.r1Surface()->name();
        if(measRobot.surfaceHasForceSensor(contact.r1Surface()->name()))
        {
          const auto & fs = measRobot.surfaceForceSensor(surfaceName);
          mapContacts_.insertContact(fs.name(), surfaceName);
          ContactWithSensor & contactWS = mapContacts_.contact(surfaceName);
          contactWS.forceNorm_ = fs.wrenchWithoutGravity(measRobot).force().norm();
          if(contactWS.forceNorm_ > contactDetectionThreshold_)
          {
            // the contact is added to the map of contacts using the name of the associated sensor
            contactsFound_.insert(contactWS.id());
          }
        }
        else
        {
          const auto & ifs = measRobot.indirectSurfaceForceSensor(surfaceName);
          mapContacts_.insertContact(ifs.name(), surfaceName);
          ContactWithSensor & contactWS = mapContacts_.contact(surfaceName);
          contactWS.forceNorm_ = ifs.wrenchWithoutGravity(measRobot).force().norm();
          if(contactWS.forceNorm_ > contactDetectionThreshold_)
          {
            // the contact is added to the map of contacts using the name of the associated sensor

            contactsFound_.insert(contactWS.id());
          }
        }
      }
    }
    else if(ctl.robots().robot(contact.r2Index()).name() == measRobot.name())
    {
      if(ctl.robots().robot(contact.r1Index()).mb().joint(0).type() == rbd::Joint::Fixed)
      {
        const std::string & surfaceName = contact.r2Surface()->name();
        if(measRobot.surfaceHasForceSensor(contact.r2Surface()->name()))
        {
          const auto & fs = measRobot.surfaceForceSensor(surfaceName);
          mapContacts_.insertContact(fs.name(), surfaceName);
          ContactWithSensor & contactWS = mapContacts_.contact(surfaceName);
          contactWS.forceNorm_ = fs.wrenchWithoutGravity(measRobot).force().norm();
          if(contactWS.forceNorm_ > contactDetectionThreshold_)
          {

            // the contact is added to the map of contacts using the name of the associated surface
            contactsFound_.insert(contactWS.id());
          }
        }
        else
        {
          const auto & ifs = measRobot.indirectSurfaceForceSensor(surfaceName);
          mapContacts_.insertContact(ifs.name(), surfaceName);
          ContactWithSensor & contactWS = mapContacts_.contact(surfaceName);
          contactWS.forceNorm_ = ifs.wrenchWithoutGravity(measRobot).force().norm();
          if(contactWS.forceNorm_ > contactDetectionThreshold_)
          {
            // the contact is added to the map of contacts using the name of the associated sensor

            contactsFound_.insert(contactWS.id());
          }
        }
      }
    }
  }
}

template<typename ContactWithSensorT>
void ContactsManager<ContactWithSensorT>::findContactsFromSurfaces(const mc_control::MCController & ctl,
                                                                   const std::string & robotName)
{
  const auto & measRobot = ctl.robot(robotName);

  contactsFound_.clear();

  for(auto & contact : mapContacts_.contacts())
  {
    const std::string & fsName = contact.second.forceSensorName();
    const mc_rbdyn::ForceSensor forceSensor = measRobot.forceSensor(fsName);

    contact.second.forceNorm_ = forceSensor.wrenchWithoutGravity(measRobot).force().norm();
    if(contact.second.forceNorm_ > contactDetectionThreshold_)
    {
      //  the contact is added to the map of contacts using the name of the associated surface
      contactsFound_.insert(contact.second.id());
    }
  }
}

template<typename ContactWithSensorT>
void ContactsManager<ContactWithSensorT>::findContactsFromSensors(const mc_control::MCController & ctl,
                                                                  const std::string & robotName)
{
  const auto & measRobot = ctl.robot(robotName);

  contactsFound_.clear();

  for(auto & contact : mapContacts_.contacts())
  {
    const std::string & fsName = contact.second.forceSensorName();
    const mc_rbdyn::ForceSensor forceSensor = measRobot.forceSensor(fsName);
    contact.second.forceNorm_ = forceSensor.wrenchWithoutGravity(measRobot).force().norm();
    if(contact.second.forceNorm_ > contactDetectionThreshold_)
    {
      // the contact is added to the map of contacts using the name of the associated sensor
      contactsFound_.insert(contact.second.id());
    }
  }
}

template<typename ContactWithSensorT>
void ContactsManager<ContactWithSensorT>::updateContacts()
{
  /** Debugging output **/
  if(verbose_ && contactsFound_ != oldContacts_)
    mc_rtc::log::info("[{}] Contacts changed: {}", observerName_, set_to_string(contactsFound_));

  for(const auto & foundContact : contactsFound_)
  {
    if(oldContacts_.find(foundContact)
       != oldContacts_.end()) // checks if the contact was already set on the last iteration
    { contact(foundContact).wasAlreadySet_ = true; }
    else // the contact was not set on the last iteration
    {
      contact(foundContact).wasAlreadySet_ = false;
      contact(foundContact).isSet_ = true;
    }
  }
  // List of the contact that were set on last iteration but are not set anymore on the current one
  removedContacts_.clear();
  std::set_difference(oldContacts_.begin(), oldContacts_.end(), contactsFound_.begin(), contactsFound_.end(),
                      std::inserter(removedContacts_, removedContacts_.end()));

  for(const auto & removedContact : removedContacts_) { contact(removedContact).resetContact(); }
  // update the list of previously set contacts
  oldContacts_ = contactsFound_;
}

template<typename ContactWithSensorT>
std::string ContactsManager<ContactWithSensorT>::set_to_string(const ContactsSet & contactSet)
{
  if(contactSet.cbegin() == contactSet.cend()) { return ""; }
  std::ostringstream out;
  out.precision(std::numeric_limits<int>::digits10);
  out << std::fixed << mapContacts_.getNameFromNum(*contactSet.cbegin());

  for(auto it = std::next(contactSet.cbegin()); it != contactSet.cend(); ++it)
  {
    out << ", ";
    out << std::fixed << mapContacts_.getNameFromNum(*it);
  }
  return out.str();
}

} // namespace mc_state_observation::measurements
