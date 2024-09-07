// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef ROMEA_SIMULATION_GAZEBO__GAZEBO_CONTROLLER_MANAGER_HPP_
#define ROMEA_SIMULATION_GAZEBO__GAZEBO_CONTROLLER_MANAGER_HPP_

// std
#include <memory>
#include <string>
#include <tuple>
#include <vector>

// ros2_control
#include "hardware_interface/resource_manager.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_manager/controller_spec.hpp"
#include "controller_manager/visibility_control.h"
#include "controller_manager_msgs/srv/configure_controller.hpp"
#include "controller_manager_msgs/srv/configure_start_controller.hpp"
#include "controller_manager_msgs/srv/list_controller_types.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/list_hardware_interfaces.hpp"
#include "controller_manager_msgs/srv/load_configure_controller.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/load_start_controller.hpp"
#include "controller_manager_msgs/srv/reload_controller_libraries.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/unload_controller.hpp"

// pluginlib
#include "pluginlib/class_loader.hpp"

// rclcpp
#include "rclcpp/executor.hpp"
#include "rclcpp/node.hpp"

namespace romea_simulation_gazebo_ros2_control
{

rclcpp::NodeOptions get_cm_node_options();

class GazeboControllerManager : public rclcpp::Node
{
public:
  static constexpr bool kWaitForAllResources = false;
  static constexpr auto kInfiniteTimeout = 0;

  GazeboControllerManager(
    std::unique_ptr<hardware_interface::ResourceManager> resource_manager,
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & namespace_ = "",
    const rclcpp::NodeOptions options = get_cm_node_options());

  GazeboControllerManager(
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & manager_node_name = "controller_manager",
    const std::string & namespace_ = "",
    const rclcpp::NodeOptions options = get_cm_node_options());

  virtual ~GazeboControllerManager() = default;

  controller_interface::ControllerInterfaceSharedPtr load_controller(
    const std::string & controller_name, const std::string & controller_type);

  /// load_controller loads a controller by name, the type must be defined in the parameter server.
  /**
   * \param[in] controller_name as a string.
   * \return controller
   * \see Documentation in controller_manager_msgs/LoadController.srv
   */
  controller_interface::ControllerInterfaceSharedPtr load_controller(
    const std::string & controller_name);

  controller_interface::return_type unload_controller(const std::string & controller_name);

  std::vector<controller_manager::ControllerSpec> get_loaded_controllers() const;

  template<
    typename T, typename std::enable_if<
      std::is_convertible<T *, controller_interface::ControllerInterface *>::value,
      T>::type * = nullptr>
  controller_interface::ControllerInterfaceSharedPtr add_controller(
    std::shared_ptr<T> controller, const std::string & controller_name,
    const std::string & controller_type)
  {
    controller_manager::ControllerSpec controller_spec;
    controller_spec.c = controller;
    controller_spec.info.name = controller_name;
    controller_spec.info.type = controller_type;
    return add_controller_impl(controller_spec);
  }

  /// configure_controller Configure controller by name calling their "configure" method.
  /**
   * \param[in] controller_name as a string.
   * \return configure controller response
   * \see Documentation in controller_manager_msgs/ConfigureController.srv
   */
  controller_interface::return_type configure_controller(const std::string & controller_name);

  /// switch_controller Stops some controllers and start others.
  /**
   * \param[in] start_controllers is a list of controllers to start
   * \param[in] stop_controllers is a list of controllers to stop
   * \param[in] set level of strictness (BEST_EFFORT or STRICT)
   * \see Documentation in controller_manager_msgs/SwitchController.srv
   */
  controller_interface::return_type switch_controller(
    const std::vector<std::string> & start_controllers,
    const std::vector<std::string> & stop_controllers, int strictness,
    bool start_asap = kWaitForAllResources,
    const rclcpp::Duration & timeout = rclcpp::Duration::from_nanoseconds(kInfiniteTimeout));

  void read();

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period);

  CONTROLLER_MANAGER_PUBLIC
  void write();

  /// Deterministic (real-time safe) callback group, e.g., update function.
  /**
   * Deterministic (real-time safe) callback group for the update function. Default behavior
   * is read hardware, update controller and finally write new values to the hardware.
   */
  // TODO(anyone): Due to issues with the MutliThreadedExecutor, this control loop does not rely on
  // the executor (see issue #260).
  // rclcpp::CallbackGroup::SharedPtr deterministic_callback_group_;

  // Per controller update rate support
  unsigned int get_update_rate() const;

protected:
  void init_services();

  controller_interface::ControllerInterfaceSharedPtr add_controller_impl(
    const controller_manager::ControllerSpec & controller);

  void manage_switch();

  void stop_controllers();

  void start_controllers();

  void start_controllers_asap();

  void list_controllers_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> response);

  void list_controller_types_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListControllerTypes::Response> response);

  void list_hardware_interfaces_srv_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ListHardwareInterfaces::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ListHardwareInterfaces::Response> response);

  void load_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::LoadController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::LoadController::Response> response);

  void configure_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ConfigureController::Response> response);

  void load_and_configure_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::LoadConfigureController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::LoadConfigureController::Response> response);

  void load_and_start_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::LoadStartController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::LoadStartController::Response> response);

  void configure_and_start_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ConfigureStartController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ConfigureStartController::Response> response);

  void reload_controller_libraries_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::ReloadControllerLibraries::Response> response);

  void switch_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response);

  void unload_controller_service_cb(
    const std::shared_ptr<controller_manager_msgs::srv::UnloadController::Request> request,
    std::shared_ptr<controller_manager_msgs::srv::UnloadController::Response> response);

  // Per controller update rate support
  unsigned int update_loop_counter_ = 0;
  unsigned int update_rate_ = 100;

private:
  std::vector<std::string> get_controller_names();

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_;

  std::shared_ptr<rclcpp::Executor> executor_;

  std::shared_ptr<pluginlib::ClassLoader<controller_interface::ControllerInterface>> loader_;

  /// Best effort (non real-time safe) callback group, e.g., service callbacks.
  /**
   * Best effort (non real-time safe) callback group for callbacks that can possibly break
   * real-time requirements, for example, service callbacks.
   */
  rclcpp::CallbackGroup::SharedPtr best_effort_callback_group_;

  /**
   * The RTControllerListWrapper class wraps a double-buffered list of controllers
   * to avoid needing to lock the real-time thread when switching controllers in
   * the non-real-time thread.
   *
   * There's always an "updated" list and an "outdated" one
   * There's always an "used by rt" list and an "unused by rt" list
   *
   * The updated state changes on the switch_updated_list()
   * The rt usage state changes on the update_and_get_used_by_rt_list()
   */
  class RTControllerListWrapper
  {
    // *INDENT-OFF*
  public:
    // *INDENT-ON*
    /// update_and_get_used_by_rt_list Makes the "updated" list the "used by rt" list
    /**
     * \warning Should only be called by the RT thread, no one should modify the
     * updated list while it's being used
     * \return reference to the updated list
     */
    std::vector<controller_manager::ControllerSpec> & update_and_get_used_by_rt_list();

    /**
     * get_unused_list Waits until the "outdated" and "unused by rt"
     * lists match and returns a reference to it
     * This referenced list can be modified safely until switch_updated_controller_list()
     * is called, at this point the RT thread may start using it at any time
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by rt list
     */
    std::vector<controller_manager::ControllerSpec> & get_unused_list(
      const std::lock_guard<std::recursive_mutex> & guard);

    /// get_updated_list Returns a const reference to the most updated list.
    /**
     * \warning May or may not being used by the realtime thread, read-only reference for safety
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by rt list
     */
    const std::vector<controller_manager::ControllerSpec> & get_updated_list(
      const std::lock_guard<std::recursive_mutex> & guard) const;

    /**
     * switch_updated_list Switches the "updated" and "outdated" lists, and waits
     *  until the RT thread is using the new "updated" list.
     * \param[in] guard Guard needed to make sure the caller is the only one accessing the unused by rt list
     */
    void switch_updated_list(const std::lock_guard<std::recursive_mutex> & guard);

    // Mutex protecting the controllers list
    // must be acquired before using any list other than the "used by rt"
    mutable std::recursive_mutex controllers_lock_;

    // *INDENT-OFF*
  private:
    // *INDENT-ON*
    /// get_other_list get the list not pointed by index
    /**
     * \param[in] index int
     */
    int get_other_list(int index) const;

    void wait_until_rt_not_using(
      int index, std::chrono::microseconds sleep_delay = std::chrono::microseconds(200)) const;

    std::vector<controller_manager::ControllerSpec> controllers_lists_[2];
    /// The index of the controller list with the most updated information
    int updated_controllers_index_ = 0;
    /// The index of the controllers list being used in the real-time thread.
    int used_by_realtime_controllers_index_ = -1;
  };

  RTControllerListWrapper rt_controllers_wrapper_;
  /// mutex copied from ROS1 Control, protects service callbacks
  /// not needed if we're guaranteed that the callbacks don't come from multiple threads
  std::mutex services_lock_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr
    list_controllers_service_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllerTypes>::SharedPtr
    list_controller_types_service_;
  rclcpp::Service<controller_manager_msgs::srv::ListHardwareInterfaces>::SharedPtr
    list_hardware_interfaces_service_;
  rclcpp::Service<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::ConfigureController>::SharedPtr
    configure_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::LoadConfigureController>::SharedPtr
    load_and_configure_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::LoadStartController>::SharedPtr
    load_and_start_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::ConfigureStartController>::SharedPtr
    configure_and_start_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::ReloadControllerLibraries>::SharedPtr
    reload_controller_libraries_service_;
  rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_service_;
  rclcpp::Service<controller_manager_msgs::srv::UnloadController>::SharedPtr
    unload_controller_service_;

  std::vector<std::string> start_request_, stop_request_;
  std::vector<std::string> start_command_interface_request_, stop_command_interface_request_;

  struct SwitchParams
  {
    bool do_switch = {false};
    bool started = {false};
    rclcpp::Time init_time = {rclcpp::Time::max()};

    // Switch options
    int strictness = {0};
    bool start_asap = {false};
    rclcpp::Duration timeout = rclcpp::Duration{0, 0};
  };

  SwitchParams switch_params_;

  static std::mutex context_mutex_;
};

}  // namespace romea_simulation_gazebo_ros2_control

#endif  // ROMEA_SIMULATION_GAZEBO__GAZEBO_CONTROLLER_MANAGER_HPP_
