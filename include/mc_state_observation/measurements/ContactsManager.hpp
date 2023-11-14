#pragma once
#include <mc_state_observation/measurements/ContactsManager.h>

namespace mc_state_observation::measurements
{

///////////////////////////////////////////////////////////////////////
/// ------------------------------Contacts-----------------------------
///////////////////////////////////////////////////////////////////////

template<typename ContactWithSensorT>
void ContactsManager<ContactWithSensorT>::init(const std::string & observerName, bool verbose)
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
                                                        double contactDetectionThreshold)
{
  contactsDetectionMethod_ = contactsDetection;
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
        addContactToManager(fsName, surface);
        addContactToGui(ctl, fsName);
      }
      else // if the surface is not associated to a force sensor, we will fetch the force sensor indirectly attached to
           // the surface
      {
        const mc_rbdyn::ForceSensor & forceSensor = robot.indirectSurfaceForceSensor(surface);
        const std::string & fsName = forceSensor.name();
        addContactToManager(fsName, surface);
        addContactToGui(ctl, fsName);
      }
    }
  }

  for(auto const & contactSensorDisabledInit : contactsSensorDisabledInit)
  {
    contact(contactSensorDisabledInit).sensorEnabled_ = false;
  }
}

template<typename ContactWithSensorT>
void ContactsManager<ContactWithSensorT>::initDetection(const mc_control::MCController & ctl,
                                                        const std::string & robotName,
                                                        ContactsDetection contactsDetection,
                                                        const std::vector<std::string> & contactsSensorDisabledInit,
                                                        double contactDetectionThreshold,
                                                        const std::vector<std::string> & forceSensorsToOmit)
{
  contactsDetectionMethod_ = contactsDetection;
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
      {
        continue;
      }
      const std::string & fsName = forceSensor.name();

      addContactToManager(fsName);
      addContactToGui(ctl, fsName);
    }
  }

  for(auto const & contactSensorDisabledInit : contactsSensorDisabledInit)
  {
    contact(contactSensorDisabledInit).sensorEnabled_ = false;
  }
}

template<typename ContactWithSensorT>
void ContactsManager<ContactWithSensorT>::addContactToGui(const mc_control::MCController & ctl,
                                                          const std::string & name)
{
  ctl.gui()->addElement(
      {observerName_, "Contacts"},
      mc_rtc::gui::Checkbox(
          name + " : " + (contact(name).isSet_ ? "Contact is set" : "Contact is not set") + ": Use wrench sensor: ",
          [this, name]() { return contact(name).sensorEnabled_; },
          [this, name]()
          {
            contact(name).sensorEnabled_ = !contact(name).sensorEnabled_;
            std::cout << std::endl
                      << "Enable / disable :" + name + " " + std::to_string(contact(name).sensorEnabled_) << std::endl;
          }));
}

template<typename ContactWithSensorT>
const std::set<int> & ContactsManager<ContactWithSensorT>::updateContacts(const mc_control::MCController & ctl,
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
  const auto & measRobot = ctl.robot(robotName);

  contactsFound_.clear();

  auto insert_contact = [this, &measRobot](const std::string & surfaceName)
  {
    if(measRobot.surfaceHasForceSensor(surfaceName))
    {
      const auto & fs = measRobot.surfaceForceSensor(surfaceName);
      ContactWithSensor & contactWS = addContactToManager(fs.name(), surfaceName);
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
      ContactWithSensor & contactWS = addContactToManager(ifs.name(), surfaceName);
      contactWS.forceNorm_ = ifs.wrenchWithoutGravity(measRobot).force().norm();
      if(contactWS.forceNorm_ > contactDetectionThreshold_)
      {
        // the contact is added to the map of contacts using the name of the associated sensor
        contactsFound_.insert(contactWS.id());
      }
    }
  };

  for(const auto & contact : ctl.solver().contacts())
  {

    const auto & r1 = ctl.robots().robot(contact.r1Index());
    const auto & r2 = ctl.robots().robot(contact.r2Index());
    if(r1.name() == measRobot.name())
    {

      if(r2.mb().nrDof() == 0) { insert_contact(contact.r1Surface()->name()); }
    }
    else if(r2.name() == measRobot.name())
    {
      if(r1.mb().nrDof() == 0) { insert_contact(contact.r1Surface()->name()); }
    }
  }
}

template<typename ContactWithSensorT>
void ContactsManager<ContactWithSensorT>::findContactsFromSurfaces(const mc_control::MCController & ctl,
                                                                   const std::string & robotName)
{
  const auto & measRobot = ctl.robot(robotName);

  contactsFound_.clear();

  for(auto & contact : contacts())
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
  findContactsFromSurfaces(ctl, robotName);
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
    {
      contact(foundContact).wasAlreadySet_ = true;
    }
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
  out << std::fixed << getNameFromIdx(*contactSet.cbegin());

  for(auto it = std::next(contactSet.cbegin()); it != contactSet.cend(); ++it)
  {
    out << ", ";
    out << std::fixed << getNameFromIdx(*it);
  }
  return out.str();
}

} // namespace mc_state_observation::measurements
