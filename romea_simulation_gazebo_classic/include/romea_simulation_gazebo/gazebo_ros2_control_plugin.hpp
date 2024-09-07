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

#ifndef ROMEA_SIMULATION_GAZEBO__GAZEBO_ROS2_CONTROL_PLUGIN_HPP_
#define ROMEA_SIMULATION_GAZEBO__GAZEBO_ROS2_CONTROL_PLUGIN_HPP_

// std
#include <memory>
#include <string>
#include <vector>

// gazebo
#include "gazebo/common/common.hh"
#include "gazebo/physics/Model.hh"

namespace romea_simulation_gazebo_ros2_control
{
class GazeboRosControlPrivate;

class GazeboRosControlPlugin : public gazebo::ModelPlugin
{
public:
  GazeboRosControlPlugin();

  virtual ~GazeboRosControlPlugin();

  // Overloaded Gazebo entry point
  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosControlPrivate> impl_;
};
}  // namespace romea_simulation_gazebo_ros2_control

#endif  // ROMEA_SIMULATION_GAZEBO__GAZEBO_ROS2_CONTROL_PLUGIN_HPP_
