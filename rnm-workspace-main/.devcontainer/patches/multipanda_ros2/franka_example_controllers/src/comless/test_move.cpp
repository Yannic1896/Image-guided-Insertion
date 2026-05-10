#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <vector>
#include <array>
#include <cmath>
#include <memory>
#include <cstdlib>
#include <chrono>

class RobotArm
{

private:
    rclcpp::Node::SharedPtr node_;
    std::vector<std::string> joint_names_;
    unsigned int num_joints_;
    double joint_move_dist_;
    std::string command_topic_;
    long counter = 0;
    std::array<double, 7> init_position{};
    sensor_msgs::msg::JointState joint_state_msg_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub; 

public:

    // Initialize
    RobotArm(rclcpp::Node::SharedPtr node, double joint_move_dist, std::string command_topic): node_(node), num_joints_(7), joint_move_dist_(joint_move_dist), command_topic_(command_topic)
    {
        
        sensor_msgs::msg::JointState msg;

        if (rclcpp::wait_for_message<sensor_msgs::msg::JointState>(
                msg, node_, "/joint_states"))
        {
            for (size_t i = 0; i < 7; ++i) {
                init_position[msg.name[i].back()-'1'] = msg.position[i];
            }
        }
        command_pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);
    }

    void sendStepCommand()
    {
        // calculate new joint angles
        // here it is just a sine wave on the initial joint angles
        std::vector<double> goal_position;
        double delta_angle = (1 - std::cos(M_PI / 5.0 * counter/500.)) *joint_move_dist_ * M_PI / 180.;
        for (size_t i = 0; i < 7; ++i) {
          if (i == 3) {
              goal_position.push_back(init_position[i] - delta_angle);
          } else {
              goal_position.push_back(init_position[i] + delta_angle);
          }
        }
        counter++;

        // create message and publish it
        std_msgs::msg::Float64MultiArray msg;
        msg.data.clear();
        msg.data.insert(msg.data.end(), goal_position.begin(), goal_position.end());
        command_pub->publish(msg); 

    }
};

int main(int argc, char** argv)
{
    // Init the ROS node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_move_node");

    // Declare the parameter
    node->declare_parameter("joint_move_dist", 1.0);
    double joint_move_dist = node->get_parameter("joint_move_dist").as_double();

    RCLCPP_INFO(node->get_logger(), "joint_move_dist: %.2f degrees.", joint_move_dist);

    // Amount of movement in each joint
    node->declare_parameter("command_topic", "/joint_position_example_controller/joint_command");
    std::string command_topic = node->get_parameter("command_topic").as_string();

    RCLCPP_INFO(node->get_logger(), "command_topic: %s", command_topic.c_str());

    // create RobotArm object
    RobotArm arm(node, joint_move_dist, command_topic);

    // loop infinitely with a fixed frequency and send our commands
    
    rclcpp::Rate loop_rate(500);  
    while (rclcpp::ok())
    {
        arm.sendStepCommand();
        rclcpp::spin_some(node); 
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
