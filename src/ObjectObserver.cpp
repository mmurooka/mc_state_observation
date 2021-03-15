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
#include <random>

namespace
{

template <typename Derived>
Derived generate(const Derived& lower, const Derived& upper)
{
  assert(lower.size() == upper.size());

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

  Derived noise = Derived::Zero();
  for(int i=0; i<lower.size(); i++)
  {
    std::uniform_real_distribution<> dis(lower[i], upper[i]);
    noise(i) = dis(gen);
  }
  return noise;
}

sva::PTransformd apply(const sva::PTransformd& X, const Eigen::Vector3d& min_ori, const Eigen::Vector3d &max_ori, const Eigen::Vector3d& min_t, const Eigen::Vector3d& max_t)
{
  const Eigen::Vector3d& noise_t = generate(min_t, max_t);
  const Eigen::Vector3d& noise_ori = generate(min_ori, max_ori);
  const Eigen::Matrix3d& noise_R = mc_rbdyn::rpyToMat(noise_ori.x(), noise_ori.y(), noise_ori.z());

  Eigen::Vector3d t_noisy = X.translation() + noise_t;
  Eigen::Matrix3d R_noisy = (X.rotation() * noise_R).eval();
  return sva::PTransformd(R_noisy, t_noisy);
}

}

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

  desc_ = fmt::format("{} (Object: {} Topic: {}, inRobotMap: {})", name(), object_, topic_, isInRobotMap_);

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

  if(!ctl.datastore().has(object_+"::X_S_Object"))
  {
    ctl.datastore().make_call(object_+"::X_S_Object",
      [this]() -> const sva::PTransformd &
      {
        return robots_.robot(object_).posW();
      }
    );
  }

  if(!ctl.datastore().has(object_+"::X_Camera_Object_Estimated"))
  {
    ctl.datastore().make_call(object_+"::X_Camera_Object_Estimated",
      [this]() -> const sva::PTransformd &
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        return X_Camera_EstimatedObject_;
      }
    );
  }

  if(!ctl.datastore().has(object_+"::X_Camera_Object_Control"))
  {
    ctl.datastore().make_call(object_+"::X_Camera_Object_Control",
      [this, &ctl]() -> const sva::PTransformd
      {
        sva::PTransformd X_0_camera = ctl.robot(robot_).bodyPosW(camera_);
        sva::PTransformd X_0_object = ctl.robot(object_).posW();
        return X_0_object * X_0_camera.inv();
      }
    );
  }

  if(!ctl.datastore().has(object_+"::X_Camera_Object_Real"))
  {
    ctl.datastore().make_call(object_+"::X_Camera_Object_Real",
      [this, &ctl]() -> const sva::PTransformd
      {
        sva::PTransformd X_0_camera = ctl.realRobot(robot_).bodyPosW(camera_);
        sva::PTransformd X_0_object = ctl.robot(object_).posW();
        return X_0_object * X_0_camera.inv();
      }
    );
  }

  if(!ctl.datastore().has("Object::"+object_+"::Tracking::Initialization"))
  {
    ctl.datastore().make_call("Object::"+object_+"::Tracking::Initialization",
      [&ctl, this]() -> void
      {
        isInitialized_ = true;
        ctl.gui()->addElement({"Object", object_},
          mc_rtc::gui::Transform("Pose",
            [&ctl, this] () -> sva::PTransformd
            {
              auto & object = ctl.robot(object_);
              const sva::PTransformd & X_0_Object = object.posW();
              auto & robot = ctl.robot();
              const sva::PTransformd & X_0_Camera = robot.bodyPosW(camera_);
              return X_0_Object * X_0_Camera.inv();
            }
          )
        );
      }
    );
    ctl.datastore().make_call("Object::"+object_+"::Tracking::Initialization::Start",
      [&ctl, this]() -> void
      {
        isContinuousInitialized_ = true;
        isInitialized_ = true;
        ctl.gui()->addElement({"Object", object_},
          mc_rtc::gui::Transform("Pose",
            [&ctl, this] () -> sva::PTransformd
            {
              auto & object = ctl.robot(object_);
              const sva::PTransformd & X_0_Object = object.posW();
              auto & robot = ctl.robot();
              const sva::PTransformd & X_0_Camera = robot.bodyPosW(camera_);
              return X_0_Object * X_0_Camera.inv();
            }
          )
        );
      }
    );
    ctl.datastore().make_call("Object::"+object_+"::Tracking::Initialization::Stop",
      [&ctl, this]() -> void
      {
        isContinuousInitialized_ = false;
        isInitialized_ = false;
        ctl.gui()->removeElement({"Object", object_}, "Pose");
      }
    );
  }

  if(isInitialized_)
  {
    ctl.gui()->removeElement({"Object", object_}, "Pose");
    isInitialized_ = false;
  }

  if(isContinuousInitialized_)
  {
    isInitialized_ = true;
    ctl.gui()->addElement({"Object", object_},
      mc_rtc::gui::Transform("Pose",
        [&ctl, this] () -> sva::PTransformd
        {
          auto & object = ctl.robot(object_);
          const sva::PTransformd & X_0_Object = object.posW();
          auto & robot = ctl.robot();
          const sva::PTransformd & X_0_Camera = robot.bodyPosW(camera_);
          return X_0_Object * X_0_Camera.inv();
        }
      )
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
    const sva::PTransformd & X_0_EstimatedObject = X_Camera_EstimatedObject;
    const auto & real_robot = ctl.realRobot();
    const sva::PTransformd & X_0_FF = real_robot.posW();
    const sva::PTransformd & X_0_Camera = real_robot.bodyPosW(camera_);
    const sva::PTransformd X_Camera_FF = X_0_FF * X_0_Camera.inv();
    const sva::PTransformd X_0_FF_sensor(ctl.robot().bodySensor().orientation(), ctl.robot().bodySensor().position());
    const sva::PTransformd X_0_Camera_sensor = X_Camera_FF.inv() * X_0_FF_sensor;
    X_Camera_EstimatedObject = X_0_EstimatedObject * X_0_Camera_sensor.inv();
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      X_Camera_EstimatedObject_ = X_Camera_EstimatedObject;
    }
  }
  const sva::PTransformd X_0_EstimatedObject =  X_Camera_EstimatedObject * X_0_Camera;
  object.posW(X_0_EstimatedObject);
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
  logger.addLogEntry(category+"_posW_in_SLAM", [this]() { return robots_.robot(object_).posW(); });
  logger.addLogEntry(category+"_X_Camera_Object_Estimated", [this]() { return X_Camera_EstimatedObject_; });
  logger.addLogEntry(category+"_X_Camera_Object_Real",
    [this, &ctl]() -> const sva::PTransformd
    {
      sva::PTransformd X_0_camera = ctl.realRobot(robot_).bodyPosW(camera_);
      sva::PTransformd X_0_object = ctl.robot(object_).posW();
      return X_0_object * X_0_camera.inv();
    }
  );
  logger.addLogEntry(category+"_X_Camera_Object_Control",
    [this, &ctl]() -> const sva::PTransformd
    {
      sva::PTransformd X_0_camera = ctl.robot(robot_).bodyPosW(camera_);
      sva::PTransformd X_0_object = ctl.robot(object_).posW();
      return X_0_object * X_0_camera.inv();
    }
  );
}

void ObjectObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category+"_posW");
  logger.removeLogEntry(category+"_posW_in_SLAM");
  logger.removeLogEntry(category+"_X_Camera_Object_Estimated");
  logger.removeLogEntry(category+"_X_Camera_Object_Real");
  logger.removeLogEntry(category+"_X_Camera_Object_Control");
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
  std::vector<std::string> categoryPose = category;
  categoryPose.push_back("Tracking System Communication");
  gui.addElement(categoryPose,
      mc_rtc::gui::Label("Status", [this] () -> std::string { return ( isInitialized_ ? "Enable" : "Disable"); }),
      mc_rtc::gui::Button("Initialization", [&ctl, this] () -> void { ctl.datastore().call("Object::"+object_+"::Tracking::Initialization"); }),
      mc_rtc::gui::Button("Start", [&ctl, this] () -> void { ctl.datastore().call("Object::"+object_+"::Tracking::Initialization::Start"); }),
      mc_rtc::gui::Button("Stop", [&ctl, this] () -> void { ctl.datastore().call("Object::"+object_+"::Tracking::Initialization::Stop"); })
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
