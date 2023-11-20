#pragma once
#include <mc_state_observation/measurements/ContactsManager.h>

namespace mc_state_observation::measurements
{

///////////////////////////////////////////////////////////////////////
/// ------------------------------Contacts-----------------------------
///////////////////////////////////////////////////////////////////////

template<typename ContactT>
void ContactsManager<ContactT>::init(const mc_control::MCController & ctl,
                                     const std::string & robotName,
                                     Configuration conf)
{
  std::visit([this, &ctl, &robotName](const auto & c) { init_manager(ctl, robotName, c); }, conf);
}

template<typename ContactT>
void ContactsManager<ContactT>::init_manager(const mc_control::MCController & ctl,
                                             const std::string & robotName,
                                             const ContactsManagerSurfacesConfiguration & conf)
{
  observerName_ = conf.observerName_;
  verbose_ = conf.verbose_;

  contactsDetectionMethod_ = Surfaces;
  contactsFinder_ = &ContactsManager<ContactT>::findContactsFromSurfaces;

  contactDetectionThreshold_ = conf.contactDetectionThreshold_;
  surfacesForContactDetection_ = conf.surfacesForContactDetection_;

  const auto & robot = ctl.robot(robotName);

  if(surfacesForContactDetection_.size() == 0)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "You selected the contacts detection using surfaces but didn't add the list of surfaces, please add it using "
        "the variable surfacesForContactDetection");
  }

  for(const std::string & surface : surfacesForContactDetection_)
  {
    // if the surface is associated to a force sensor (for example LeftFootCenter or RightFootCenter)
    if(robot.surfaceHasForceSensor(surface))
    {
      const mc_rbdyn::ForceSensor & forceSensor = robot.surfaceForceSensor(surface);
      const std::string & fsName = forceSensor.name();
      addContactToManager(fsName, surface);
    }
    else // if the surface is not associated to a force sensor, we will fetch the force sensor indirectly attached to
         // the surface
    {
      const mc_rbdyn::ForceSensor & forceSensor = robot.indirectSurfaceForceSensor(surface);
      const std::string & fsName = forceSensor.name();
      addContactToManager(fsName, surface);
    }
  }
}
template<typename ContactT>
void ContactsManager<ContactT>::init_manager(const mc_control::MCController & ctl,
                                             const std::string & robotName,
                                             const ContactsManagerSensorsConfiguration & conf)
{
  observerName_ = conf.observerName_;
  verbose_ = conf.verbose_;

  contactsDetectionMethod_ = Sensors;
  contactsFinder_ = &ContactsManager<ContactT>::findContactsFromSensors;

  contactDetectionThreshold_ = conf.contactDetectionThreshold_;

  const auto & robot = ctl.robot(robotName);

  for(auto & forceSensor : robot.forceSensors())
  {
    if(std::find(conf.forceSensorsToOmit_.begin(), conf.forceSensorsToOmit_.end(), forceSensor.name())
       != conf.forceSensorsToOmit_.end())
    {
      continue;
    }
    const std::string & fsName = forceSensor.name();

    addContactToManager(fsName);
  }
}
template<typename ContactT>
void ContactsManager<ContactT>::init_manager(const mc_control::MCController &,
                                             const std::string &,
                                             const ContactsManagerSolverConfiguration & conf)
{
  observerName_ = conf.observerName_;
  verbose_ = conf.verbose_;

  contactsDetectionMethod_ = Solver;
  contactsFinder_ = &ContactsManager<ContactT>::findContactsFromSolver;

  contactDetectionThreshold_ = conf.contactDetectionThreshold_;
}

template<typename ContactT>
template<typename OnNewContact, typename OnMaintainedContact, typename OnRemovedContact>
const std::set<int> & ContactsManager<ContactT>::updateContacts(const mc_control::MCController & ctl,
                                                                const std::string & robotName,
                                                                OnNewContact & onNewContact,
                                                                OnMaintainedContact & onMaintainedContact,
                                                                OnRemovedContact & onRemovedContact)
{
  // Detection of the contacts depending on the configured mode
  (this->*contactsFinder_)(ctl, robotName);

  /** Debugging output **/
  if(verbose_ && contactsFound_ != oldContacts_)
    mc_rtc::log::info("[{}] Contacts changed: {}", observerName_, set_to_string(contactsFound_));

  for(const auto & foundContact : contactsFound_)
  {
    if(oldContacts_.find(foundContact)
       != oldContacts_.end()) // checks if the contact was already set on the last iteration
    {
      contact(foundContact).wasAlreadySet_ = true;
      onMaintainedContact(contact(foundContact));
    }
    else // the contact was not set on the last iteration
    {
      contact(foundContact).wasAlreadySet_ = false;
      contact(foundContact).isSet_ = true;
      onNewContact(contact(foundContact));
    }
  }
  // List of the contact that were set on last iteration but are not set anymore on the current one
  removedContacts_.clear();
  std::set_difference(oldContacts_.begin(), oldContacts_.end(), contactsFound_.begin(), contactsFound_.end(),
                      std::inserter(removedContacts_, removedContacts_.end()));

  for(const auto & removedContact : removedContacts_)
  {
    contact(removedContact).resetContact();
    onRemovedContact(contact(removedContact));
  }
  // update the list of previously set contacts
  oldContacts_ = contactsFound_;

  return contactsFound_; // list of currently set contacts
}

template<typename ContactT>
void ContactsManager<ContactT>::findContactsFromSolver(const mc_control::MCController & ctl,
                                                       const std::string & robotName)
{
  const auto & measRobot = ctl.robot(robotName);

  contactsFound_.clear();

  auto insert_contact = [this, &measRobot](const std::string & surfaceName)
  {
    const auto & fs = measRobot.surfaceHasForceSensor(surfaceName) ? measRobot.surfaceForceSensor(surfaceName)
                                                                   : measRobot.indirectSurfaceForceSensor(surfaceName);
    ContactWithSensor & contactWS = addContactToManager(fs.name(), surfaceName);
    contactWS.forceNorm_ = fs.wrenchWithoutGravity(measRobot).force().norm();
    if(contactWS.forceNorm_ > contactDetectionThreshold_)
    {
      // the contact is added to the map of contacts using the name of the associated sensor
      contactsFound_.insert(contactWS.id());
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

template<typename ContactT>
void ContactsManager<ContactT>::findContactsFromSurfaces(const mc_control::MCController & ctl,
                                                         const std::string & robotName)
{
  const auto & measRobot = ctl.robot(robotName);

  contactsFound_.clear();

  for(auto & contact : contacts())
  {
    const std::string & fsName = contact.second.forceSensor();
    const mc_rbdyn::ForceSensor & forceSensor = measRobot.forceSensor(fsName);
    contact.second.forceNorm_ = forceSensor.wrenchWithoutGravity(measRobot).force().norm();
    if(contact.second.forceNorm_ > contactDetectionThreshold_)
    {
      //  the contact is added to the map of contacts using the name of the associated surface
      contactsFound_.insert(contact.second.id());
    }
  }
}

template<typename ContactT>
void ContactsManager<ContactT>::findContactsFromSensors(const mc_control::MCController & ctl,
                                                        const std::string & robotName)
{
  findContactsFromSurfaces(ctl, robotName);
}

template<typename ContactT>
std::string ContactsManager<ContactT>::set_to_string(const ContactsSet & contactSet)
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
