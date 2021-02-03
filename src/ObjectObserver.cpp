#include <mc_observers/ObserverMacros.h>
#include <mc_control/MCController.h>
#include <Eigen/src/Geometry/Transform.h>
#include <mc_state_observation/ObjectObserver.h>
#include <mc_state_observation/gui_helpers.h>
#include <mc_rtc/ros.h>
#include <mc_rtc/version.h>
#include <SpaceVecAlg/Conversions.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <tf2_eigen/tf2_eigen.h>

namespace mc_state_observation
{

ObjectObserver::ObjectObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt), nh_(mc_rtc::ROSBridge::get_node_handle())
{}

void ObjectObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  if(config.has("Robot"))
  {
    robot_ = config("Robot")("robot", ctl.robot().name());
    camera_ = static_cast<std::string>(config("Robot")(robot_)("camera"));
    if(!ctl.robot(robot_).hasBody(camera_))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("No {} body found in {}", camera_, robot_);
    }
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Robot configuration is mandatory.", name());
  }

  if(config.has("Object"))
  {
    object_ = static_cast<std::string>(config("Object")("robot"));
    topic_ = static_cast<std::string>(config("Object")("topic"));
    isInRobotMap_ = config("Object")("inRobotMap", false);

    robots_.load(object_, ctl.robot(object_).module());
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Object configuration is mandatory.", name());
  }

  if(config.has("Publish"))
  {
    isPublished_ = config("Publish")("use", true);
  }

  subscriber_ = nh_->subscribe(topic_, 1, &ObjectObserver::callback, this);

  desc_ = fmt::format("{} (Object: {} Topic: {}, inRobotMap: {})", name(), name_, object_, topic_, isInRobotMap_);

  thread_ = std::thread(std::bind(&ObjectObserver::rosSpinner, this));
}

void ObjectObserver::reset(const mc_control::MCController &)
{}

bool ObjectObserver::run(const mc_control::MCController &)
{
  return true;
}

void ObjectObserver::update(mc_control::MCController & ctl)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if(!isNewEstimatedPose_)
    {
      return;
    }
  }

  if(!ctl.datastore().has(object_+"::Robot"))
  {
    ctl.datastore().make_call(object_+"::Robot",
      [this, &ctl]() -> const mc_rbdyn::Robot &
      {
        return ctl.realRobot(object_);
      }
    );
  }

  if(!ctl.datastore().has(object_+"::SLAM::Robot"))
  {
    ctl.datastore().make_call(object_+"::SLAM::Robot",
      [this]() -> const mc_rbdyn::Robot &
      {
        return robots_.robot(object_);
      }
    );
  }

  if(!ctl.datastore().has(object_+"::X_0_Object"))
  {
    ctl.datastore().make_call(object_+"::X_0_Object",
      [this, &ctl]() -> const sva::PTransformd &
      {
        return ctl.realRobot(object_).posW();
      }
    );
  }

  if(!ctl.datastore().has(object_+"::X_0_Object_inSLAM"))
  {
    ctl.datastore().make_call(object_+"::X_0_Object_inSLAM",
      [this]() -> const sva::PTransformd &
      {
        return robots_.robot(object_).posW();
      }
    );
  }

  if(!ctl.datastore().has(object_+"::X_Camera_Object"))
  {
    ctl.datastore().make_call(object_+"::X_Camera_Object",
      [this]() -> const sva::PTransformd &
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        return X_Camera_EstimatedObject_;
      }
    );
  }

  const auto & real_robot = ctl.realRobot(robot_);
  const sva::PTransformd X_0_Camera = real_robot.bodyPosW(camera_);
  sva::PTransformd X_Camera_EstimatedObject;
  auto & object = ctl.realRobot(object_);
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    X_Camera_EstimatedObject = X_Camera_EstimatedObject_;
  }
  if(isInRobotMap_)
  {
    object.posW(X_Camera_EstimatedObject);
    X_Camera_EstimatedObject_ = X_Camera_EstimatedObject * X_0_Camera.inv();
  }
  else
  {
    const sva::PTransformd X_0_EstimatedObject =  X_Camera_EstimatedObject * X_0_Camera;
    object.posW(X_0_EstimatedObject);
  }
  isNewEstimatedPose_ = false;
  object.forwardKinematics();

  if(ctl.datastore().has("SLAM::Robot"))
  {
    const sva::PTransformd & X_0_Camera = ctl.datastore().call<const mc_rbdyn::Robot &>("SLAM::Robot").bodyPosW(camera_);
    auto & object = robots_.robot(object_);
    object.posW(X_Camera_EstimatedObject_ * X_0_Camera);
    object.forwardKinematics();
  }

  if(isPublished_)
  {
    mc_rtc::ROSBridge::update_robot_publisher(object_+"_estimated", ctl.timeStep, object);
    if(ctl.datastore().has("SLAM::Robot"))
    {
      mc_rtc::ROSBridge::update_robot_publisher(object_+"_estimated_in_SLAM", ctl.timeStep, robots_.robot(object_));
    }
  }
}

void ObjectObserver::addToLogger(const mc_control::MCController & ctl, mc_rtc::Logger & logger, const std::string & category)
{
  logger.addLogEntry(category+"_posW", [this, &ctl]() { return ctl.realRobot(object_).posW(); });
  logger.addLogEntry(category+"_X_Camera_Object", [this, &ctl]() { return X_Camera_EstimatedObject_; });
}

void ObjectObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category+"_posW");
  logger.removeLogEntry(category+"_X_Camera_Object");
}

void ObjectObserver::addToGUI(const mc_control::MCController & ctl,
                             mc_rtc::gui::StateBuilder & gui,
                             const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;

  gui.addElement(category,
    Transform(
      "X_0_"+object_,
      [this, &ctl]() { return ctl.realRobot(object_).posW(); }
    ),
    Transform(
      "X_Camera_"+object_,
      [this]()
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        return X_Camera_EstimatedObject_;
      }
    )
  );
}

void ObjectObserver::callback(const geometry_msgs::PoseStamped & msg)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(msg.pose, affine);
  const std::lock_guard<std::mutex> lock(mutex_);
  X_Camera_EstimatedObject_ = sva::conversions::fromHomogeneous(affine.matrix());
  isNewEstimatedPose_ = true;
}

void ObjectObserver::rosSpinner()
{
  mc_rtc::log::info("[{}] rosSpinner started", name());
  ros::Rate rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  mc_rtc::log::info("[{}] rosSpinner finished", name());
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("Object", mc_state_observation::ObjectObserver)