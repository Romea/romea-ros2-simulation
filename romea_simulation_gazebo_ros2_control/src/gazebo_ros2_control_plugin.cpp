// Copyright (c) 2013, Open Source Robotics Foundation. All rights reserved.
// Copyright (c) 2013, The Johns Hopkins University. All rights reserved.
// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Gazebo plugin for ros_control that allows 'hardware_interfaces' to be plugged in
   using pluginlib
*/

// std
#include <string>
#include <memory>
#include <utility>
#include <vector>

// tinyxml
#include "tinyxml2.h" // NOLINT

// gazebo
#include "gazebo_ros/node.hpp"
#include "gazebo_ros2_control/gazebo_system.hpp"

// pluginlib
#include "pluginlib/class_loader.hpp"

// ros2_control
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// rclcpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"

// local
#include "romea_simulation_gazebo_ros2_control/gazebo_ros2_control_plugin.hpp"


#define ROS_GALACTIC 0
#define ROS_HUMBLE 1

#if ROS_DISTRO == ROS_GALACTIC
#include "romea_simulation_gazebo_ros2_control/gazebo_controller_manager.hpp"
using ControllerManager = romea_simulation_gazebo_ros2_control::GazeboControllerManager;

std::vector<const tinyxml2::XMLElement *> findChildElements(
  const tinyxml2::XMLElement * parent,
  const char * child_name)
{
  std::vector<const tinyxml2::XMLElement *> childs;
  const auto * child = parent->FirstChildElement(child_name);
  while (child) {
    childs.push_back(child);
    child = child->NextSiblingElement(child_name);
  }
  return childs;
}

#else
#include "controller_manager/controller_manager.hpp"
using ControllerManager = controller_manager::ControllerManager;
#endif


namespace romea_simulation_gazebo_ros2_control
{

class GazeboRosControlPrivate
{
public:
  GazeboRosControlPrivate() = default;

  virtual ~GazeboRosControlPrivate() = default;

  // Called by the world update start event
  void Update();

  // Called on world reset
  virtual void Reset();

  // Get the URDF XML from the parameter server
  std::string getURDF() const;

  // Node Handles
  gazebo_ros::Node::SharedPtr model_nh_;

  // Pointer to the model
  gazebo::physics::ModelPtr parent_model_;

  // Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_;

  // Interface loader
  boost::shared_ptr<pluginlib::ClassLoader<
      gazebo_ros2_control::GazeboSystemInterface>> robot_hw_sim_loader_;

  // Controller manager configuration file
  std::string controller_manager_config_file_;

  // ROS2 control configuration file
  std::string ros2_control_config_file_;

  // Executor to spin the controller
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

  // Thread where the executor will sping
  std::thread thread_executor_spin_;

  // Flag to stop the executor thread when this plugin is exiting
  bool stop_;

  // Controller manager
  std::shared_ptr<ControllerManager> controller_manager_;

  // Available controllers
  std::vector<std::shared_ptr<controller_interface::ControllerInterface>> controllers_;

  // Timing
  rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);

  // Last time the update method was called
  rclcpp::Time last_update_sim_time_ros_ = rclcpp::Time((int64_t)0, RCL_ROS_TIME);
};

GazeboRosControlPlugin::GazeboRosControlPlugin()
: impl_(std::make_unique<GazeboRosControlPrivate>())
{
}

GazeboRosControlPlugin::~GazeboRosControlPlugin()
{
  // Stop controller manager thread
  impl_->stop_ = true;
  impl_->executor_->remove_node(impl_->controller_manager_);
  impl_->executor_->cancel();
  impl_->thread_executor_spin_.join();

  // Disconnect from gazebo events
  impl_->update_connection_.reset();
}

// Overloaded Gazebo entry point
void GazeboRosControlPlugin::Load(
  gazebo::physics::ModelPtr parent,
  sdf::ElementPtr sdf)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("gazebo_ros2_control"), "Loading gazebo_ros2_control plugin");

  // Save pointers to the model
  impl_->parent_model_ = parent;

  // Initialize ROS node
  impl_->model_nh_ = gazebo_ros::Node::Get(sdf);
  auto logger = impl_->model_nh_->get_logger();

  RCLCPP_INFO(
    logger, "Starting gazebo_ros2_control plugin in namespace: %s",
    impl_->model_nh_->get_namespace());

  RCLCPP_INFO(
    logger, "Starting gazebo_ros2_control plugin in ros 2 node: %s",
    impl_->model_nh_->get_name());

  // Check that ROS has been initialized
  if (!rclcpp::ok()) {
    RCLCPP_FATAL_STREAM(
      logger,
      "A ROS node for Gazebo has not been initialized" <<
        " unable to load plugin libromea_gazebo_ros2_plugin.so");
    return;
  }

  if (sdf->HasElement("controller_manager_config_file")) {
    impl_->controller_manager_config_file_ =
      sdf->GetElement("controller_manager_config_file")->Get<std::string>();
  } else {
    RCLCPP_ERROR(logger, "No controller manager configuration file provided");
    return;
  }

  // Get ros2 control config
  if (sdf->HasElement("ros2_control_config_file")) {
    impl_->ros2_control_config_file_ =
      sdf->GetElement("ros2_control_config_file")->Get<std::string>();
  } else {
    RCLCPP_ERROR(logger, "No ros2 control configuration file provided");
    return;
  }

  // Get the Gazebo simulation period
  rclcpp::Duration gazebo_period(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(
        impl_->parent_model_->GetWorld()->Physics()->GetMaxStepSize())));

  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  // This call will block if ROS is not properly initialized.
  std::string urdf_string;
  std::vector<hardware_interface::HardwareInfo> control_hardware_info;
  try {
    urdf_string = impl_->getURDF();
    control_hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string);
  } catch (const std::runtime_error & ex) {
    RCLCPP_ERROR_STREAM(logger, "Error parsing URDF in gazebo_ros2_control plugin :" << ex.what());
    return;
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_ =
    std::make_unique<hardware_interface::ResourceManager>();

  try {
    impl_->robot_hw_sim_loader_.reset(
      new pluginlib::ClassLoader<gazebo_ros2_control::GazeboSystemInterface>(
        "gazebo_ros2_control",
        "gazebo_ros2_control::GazeboSystemInterface"));
  } catch (pluginlib::LibraryLoadException & ex) {
    RCLCPP_ERROR(logger, "Failed to create robot simulation interface loader : %s", ex.what());
  }


  for (unsigned int i = 0; i < control_hardware_info.size(); i++) {
    std::string robot_hw_sim_type_str_ = control_hardware_info[i].hardware_class_type;
    std::cout << robot_hw_sim_type_str_ << std::endl;
    auto gazeboSystem = std::unique_ptr<gazebo_ros2_control::GazeboSystemInterface>(
      impl_->robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));

    rclcpp::Node::SharedPtr node_ros2 = std::dynamic_pointer_cast<rclcpp::Node>(impl_->model_nh_);
    if (!gazeboSystem->initSim(node_ros2, impl_->parent_model_, control_hardware_info[i], sdf)) {
      RCLCPP_FATAL(logger, "Could not initialize robot simulation interface");
      return;
    }

    // auto command_interfaces = gazeboSystem->export_command_interfaces();
    // auto state_interfaces = gazeboSystem->export_command_interfaces();


    resource_manager_->import_component(std::move(gazeboSystem), control_hardware_info[i]);


#if ROS_DISTRO == ROS_HUMBLE
    rclcpp_lifecycle::State state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
    resource_manager_->set_component_state(control_hardware_info[i].name, state);
#endif
  }

#if ROS_DISTRO == ROS_GALACTIC
  // ugly code to set joint intial positions under galactic distro
  tinyxml2::XMLDocument doc;
  doc.Parse(impl_->getURDF().c_str());
  const auto * robot_it = doc.RootElement();
  for (const auto * ros2_control_it : findChildElements(robot_it, "ros2_control")) {
    for (const auto * joint_it : findChildElements(ros2_control_it, "joint")) {
      std::string joint_name = std::string(joint_it->Attribute("name")) + "/position";
      for (const auto * state_interface_it : findChildElements(joint_it, "state_interface")) {
        if (std::string(state_interface_it->Attribute("name")) == "position") {
          for (const auto * param_it : findChildElements(state_interface_it, "param")) {
            if (std::string(param_it->Attribute("name")) == "initial_value" &&
              resource_manager_->state_interface_exists(joint_name))
            {
              resource_manager_->claim_command_interface(joint_name).set_value(
                param_it->FloatText());
            }
          }
        }
      }
    }
  }
#endif

  impl_->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

#if ROS_DISTRO == ROS_GALACTIC
  rclcpp::NodeOptions options = get_cm_node_options();
#else
  rclcpp::NodeOptions options = controller_manager::get_cm_node_options();
#endif

  options.arguments(
    {"--ros-args", "--params-file",
      impl_->controller_manager_config_file_.c_str()});

  // Create the controller manager
  RCLCPP_INFO(logger, "Loading controller_manager");
  impl_->controller_manager_.reset(
    new ControllerManager(
      std::move(resource_manager_),
      impl_->executor_,
      "controller_manager",
      impl_->model_nh_->get_namespace(),
      options));
  impl_->executor_->add_node(impl_->controller_manager_);

  if (!impl_->controller_manager_->has_parameter("update_rate")) {
    RCLCPP_ERROR_STREAM(logger, "controller manager doesn't have an update_rate parameter");
    return;
  }

  auto cm_update_rate = impl_->controller_manager_->get_parameter("update_rate").as_int();
  impl_->control_period_ = rclcpp::Duration(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / static_cast<double>(cm_update_rate))));

  // Check the period against the simulation period
  if (impl_->control_period_ < gazebo_period) {
    RCLCPP_ERROR_STREAM(
      logger,
      "Desired controller update period (" << impl_->control_period_.seconds() <<
        " s) is faster than the gazebo simulation period (" << gazebo_period.seconds() <<
        " s).");
  } else if (impl_->control_period_ > gazebo_period) {
    RCLCPP_WARN_STREAM(
      logger,
      " Desired controller update period (" << impl_->control_period_.seconds() <<
        " s) is slower than the gazebo simulation period (" << gazebo_period.seconds() <<
        " s).");
  }

  impl_->stop_ = false;
  auto spin = [this]()
    {
      while (rclcpp::ok() && !impl_->stop_) {
        impl_->executor_->spin_once();
      }
    };
  impl_->thread_executor_spin_ = std::thread(spin);

  // Listen to the update event. This event is broadcast every simulation iteration.
  impl_->update_connection_ =
    gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(
      &GazeboRosControlPrivate::Update,
      impl_.get()));


  RCLCPP_INFO(logger, "Loaded gazebo_ros2_control.");
}

// Called by the world update start event
void GazeboRosControlPrivate::Update()
{
  // Get the simulation time and period
  gazebo::common::Time gz_time_now = parent_model_->GetWorld()->SimTime();
  rclcpp::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec, RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  if (sim_period >= control_period_) {
#if ROS_DISTRO == ROS_GALACTIC
    controller_manager_->read();
#else
    controller_manager_->read(sim_time_ros, sim_period);
#endif
    controller_manager_->update(sim_time_ros, sim_period);
    last_update_sim_time_ros_ = sim_time_ros;
  }

  // Always set commands on joints, otherwise at low control frequencies the joints tremble
  // as they are updated at a fraction of gazebo sim time
#if ROS_DISTRO == ROS_GALACTIC
  controller_manager_->write();
#else
  controller_manager_->write(sim_time_ros, sim_period);
#endif
}

// Called on world reset
void GazeboRosControlPrivate::Reset()
{
  // Reset timing variables to not pass negative update periods to controllers on world reset
  last_update_sim_time_ros_ = rclcpp::Time((int64_t)0, RCL_ROS_TIME);
}

// Get the URDF XML from the parameter server
std::string GazeboRosControlPrivate::getURDF() const
{
  tinyxml2::XMLDocument urdf_xml;
  urdf_xml.LoadFile(ros2_control_config_file_.c_str());
  tinyxml2::XMLPrinter printer;
  urdf_xml.Print(&printer);
  return printer.CStr();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosControlPlugin)
}   // namespace romea_simulation_gazebo_ros2_control
