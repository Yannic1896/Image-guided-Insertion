// Copyright (c) 2021 Franka Emika GmbH
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

#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "realtime_tools/realtime_publisher.hpp"


#include "franka_example_controllers/comless/Trajectory.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

class JointPositionExampleController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  const int num_joints = 7;
  Vector7d q_;
  Vector7d dq_;
  rclcpp::Duration elapsed_time_ = rclcpp::Duration(0, 0);

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr trajectory_sub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::UInt64>> trajectory_finished_pub_;
  
  std::shared_ptr<Trajectory> command_;
  std::mutex check_mutex_;
  std::array<double, 7> current_pose_{};
  const double max_joint_speed = 1.5;
  double last_diff_ = 0;
  rclcpp::Time last_command_time_;
  bool executing_trajectory_ = false;

  void updateJointStates();

  void commandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr & msg);
  void trajectoryCallback(const std_msgs::msg::Float64MultiArray::SharedPtr & msg);
  [[nodiscard]] bool validateJointSpeed(double previousCommand, double commanded) const;
  std::shared_ptr<Trajectory> createQueueFromMsg(const std_msgs::msg::Float64MultiArray::SharedPtr & msg, bool should_send_completed=false, bool not_trajectory=true);
       

};

}  // namespace franka_example_controllers
