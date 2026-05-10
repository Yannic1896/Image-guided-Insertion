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

#include "franka_example_controllers/comless/joint_position_example_controller.hpp"
#include "franka_example_controllers/comless/Trajectory.hpp"

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>


#include <memory>
#include <atomic> 
#include <mutex>
#include <array>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/u_int64.hpp"


namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointPositionExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointPositionExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type JointPositionExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
      elapsed_time_ = elapsed_time_ + period;
      auto current_command = std::atomic_load(&command_);
      bool current_command_empty = current_command->get().empty();
      if (current_command_empty) {
        for (const auto &angle : current_pose_) {
          current_command->get().push(angle);
        }
      } 
      else if (current_command->getShouldSendCompleted()) {
        executing_trajectory_ = true;
      }

      for (size_t i = 0; i < 7; i++) {
        double cmd = current_command->get().front();
        command_interfaces_[i].set_value(cmd);
        { // update internal pose
          std::lock_guard<std::mutex> guard(check_mutex_);
          current_pose_[i] = cmd;
        }
        current_command->get().pop();
      }

      if (current_command_empty && executing_trajectory_) {
        executing_trajectory_ = false;

        if (trajectory_finished_pub_->trylock()) {
          trajectory_finished_pub_->msg_.data = current_command->getId();
          trajectory_finished_pub_->unlockAndPublish();
        }
      } 

  return controller_interface::return_type::OK;
}

CallbackReturn
JointPositionExampleController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
   
    command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/joint_position_example_controller/joint_command", 1, 
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
    	this->commandCallback(msg);
    });
    
    trajectory_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/joint_position_example_controller/joint_trajectory_command", 1, 
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
    	this->trajectoryCallback(msg);
    });

    trajectory_finished_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::UInt64>>(
        get_node()->create_publisher<std_msgs::msg::UInt64>(
            "trajectory_finished",
            rclcpp::QoS(1)
        )
    );


  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointPositionExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  return CallbackReturn::SUCCESS;
}

CallbackReturn 
JointPositionExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  for (int i = 0; i < 7; ++i) {
    current_pose_[i] = q_(i);
    RCLCPP_INFO( get_node()->get_logger(),"Joint %d position: %f", i, current_pose_[i] );
  }
  elapsed_time_ = rclcpp::Duration(0, 0);
  last_command_time_ = get_node()->now();

  command_ = std::make_shared<Trajectory>();

  return CallbackReturn::SUCCESS;
}

void JointPositionExampleController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

void JointPositionExampleController::trajectoryCallback(const std_msgs::msg::Float64MultiArray::SharedPtr & msg) {
        auto tmp = createQueueFromMsg(msg, true, false);
        std::atomic_store(&command_, tmp);
}


void JointPositionExampleController::commandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr & msg) {
        auto tmp = createQueueFromMsg(msg, false, true);
        std::atomic_store(&command_, tmp);
}

    
std::shared_ptr<Trajectory> JointPositionExampleController::createQueueFromMsg(
    const std_msgs::msg::Float64MultiArray::SharedPtr & msg,
    bool should_send_completed, bool not_trajectory)
  {
      auto tmp = std::make_shared<Trajectory>(should_send_completed);
      if (msg->data.size() % 7 != 0) {
          RCLCPP_ERROR(
              get_node()->get_logger(),
              "Could not set command. Did not receive multiple of 7 angles (%zu)",
              msg->data.size());
      }
      std::array<double, 7> previous_command;
      {
        if(not_trajectory) {
          std::lock_guard<std::mutex> guard(check_mutex_);
          previous_command = current_pose_;
        }
        else {
          updateJointStates();
          for (int i = 0; i < 7; ++i) {
            previous_command[i] = q_(i);
          }
        }
      }
      for (size_t i = 0; i < msg->data.size(); ++i) {
          if (!validateJointSpeed(previous_command[i % 7], msg->data[i])) {
              RCLCPP_WARN(
                  get_node()->get_logger(),
                  "Large motion command detected at joint %lu (current: %f - commanded: %f)!\n"
                  "This would likely cause an emergency stop with the real hardware\n"
                  "You need to command to poses that are reachable within the 1ms timeframe",
                  i % 7, previous_command[i % 7], msg->data[i]);
          }
          if ((i + 1) % 7 == 0) {
              std::copy(msg->data.begin() + i - 6,
                        msg->data.begin() + i + 1,
                        previous_command.begin());
          }
      }
      // Time diff check
      rclcpp::Time now = get_node()->now();
      double diff = (now - last_command_time_).seconds();
      if (last_diff_ > 0) {
          diff = (diff + last_diff_ * 9) / 10;
      }
      last_diff_ = diff;
      if (diff < 0.0005) {
          RCLCPP_WARN(
              get_node()->get_logger(),
              "Received commands too quickly (%f [s]). Real hardware only handles joint command every 1 ms",
              diff);
      }
      for (const auto & angle : msg->data) {
          tmp->get().push(angle);
      }
      last_command_time_ = now;
      return tmp;
}


bool JointPositionExampleController::validateJointSpeed(double previous_command, double commanded) const {
        return std::abs(previous_command - commanded) * 1000 < max_joint_speed;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerInterface)
