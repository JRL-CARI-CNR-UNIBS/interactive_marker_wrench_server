// Copyright 2011 Willow Garage, Inc.
// Copyright 2013 Mike Purvis
// Copyright 2021 Clearpath Robotics Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <string>
#include <map>
#include <memory>

namespace interactive_marker_wrench_server {

class WrenchServerNode : public rclcpp::Node {
public:
  WrenchServerNode();

  ~WrenchServerNode() = default;

  void processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &
    feedback);

private:
  void getParameters();
  void createInteractiveMarkers();
  void stampAndPublish(geometry_msgs::msg::Wrench& msg);

  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_pub;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_stamped_pub;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server;

  std::map<std::string, double> linear_drive_scale_map;
  std::map<std::string, double> max_positive_linear_force_map;
  std::map<std::string, double> max_negative_linear_force_map;

  bool use_stamped_msgs;

  double angular_drive_scale;
  double max_angular_force;
  double marker_size_scale;

  std::string topic;
  std::string link_name;
  std::string robot_name;
}; // class WrenchServerNode

WrenchServerNode::WrenchServerNode()
    : rclcpp::Node("wrench_server_node",
                   rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)),
      server(std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "wrench_server", get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
        get_node_topics_interface(), get_node_services_interface())) {
  getParameters();
  if (use_stamped_msgs)
  {
    wrench_stamped_pub = create_publisher<geometry_msgs::msg::WrenchStamped>("", 1);
  } 
  else
  {
    wrench_pub = create_publisher<geometry_msgs::msg::Wrench>(topic, 1);
  }
  createInteractiveMarkers();
  RCLCPP_INFO(get_logger(), "[interactive_marker_wrench_server] Initialized.");
}

void WrenchServerNode::getParameters() {
  rclcpp::Parameter link_name_param;
  rclcpp::Parameter robot_name_param;
  rclcpp::Parameter use_stamped_msgs_param;

  topic = this->get_parameter_or("topic", std::string("wrench_cmd"));

  if (this->get_parameter("link_name", link_name_param))
  {
    link_name = link_name_param.as_string();
  }
  else
  {
    link_name = "base_link";
  }

  if (this->get_parameter("robot_name", robot_name_param))
  {
    robot_name = robot_name_param.as_string();
  }
  else
  {
    robot_name = "robot";
  }

  if (this->get_parameter("use_stamped_msgs", use_stamped_msgs_param))
  {
    use_stamped_msgs = use_stamped_msgs_param.as_bool();
  }
  else
  {
    use_stamped_msgs = false;
  }

  // Ensure parameters are loaded correctly, otherwise, manually set values for linear config
  if (this->get_parameters("linear_scale", linear_drive_scale_map))
  {
    this->get_parameters("max_positive_linear_force", max_positive_linear_force_map);
    this->get_parameters("max_negative_linear_force", max_negative_linear_force_map);
  }
  else
  {
    linear_drive_scale_map["x"] = 1.0;
    max_positive_linear_force_map["x"] = 1.0;
    max_negative_linear_force_map["x"] = -1.0;
  }

  angular_drive_scale = 2.2;
  max_angular_force = 2.2;
  marker_size_scale = 1.0;
}

void WrenchServerNode::createInteractiveMarkers() {
  visualization_msgs::msg::InteractiveMarker interactive_marker;
  interactive_marker.header.frame_id = link_name;
  interactive_marker.name = robot_name + "_wrench_marker";
  interactive_marker.description = "wrench controller for " + robot_name;
  interactive_marker.scale = marker_size_scale;

  visualization_msgs::msg::InteractiveMarkerControl control;

  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;

  if (linear_drive_scale_map.find("x") != linear_drive_scale_map.end())
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker.controls.push_back(control);
  }

  if (linear_drive_scale_map.find("y") != linear_drive_scale_map.end())
  {
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker.controls.push_back(control);
  }

  if (linear_drive_scale_map.find("z") != linear_drive_scale_map.end())
  {
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker.controls.push_back(control);
  }

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  interactive_marker.controls.push_back(control);

  server->insert(interactive_marker);
  server->setCallback(interactive_marker.name, std::bind(&WrenchServerNode::processFeedback, this, std::placeholders::_1));
  server->applyChanges();
}

void WrenchServerNode::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
  geometry_msgs::msg::Wrench wrench_msg;

  // Handle angular change (yaw is the only direction in which you can rotate)
  double yaw = tf2::getYaw(feedback->pose.orientation);
  wrench_msg.torque.z = angular_drive_scale * yaw;
  wrench_msg.torque.z = std::min(wrench_msg.torque.z, max_angular_force);
  wrench_msg.torque.z = std::max(wrench_msg.torque.z, -max_angular_force);

  if (linear_drive_scale_map.find("x") != linear_drive_scale_map.end())
  {
    wrench_msg.force.x = linear_drive_scale_map["x"] * feedback->pose.position.x;
    wrench_msg.force.x = std::min(wrench_msg.force.x, max_positive_linear_force_map["x"]);
    wrench_msg.force.x = std::max(wrench_msg.force.x, max_negative_linear_force_map["x"]);
  }

  if (linear_drive_scale_map.find("y") != linear_drive_scale_map.end())
  {
    wrench_msg.force.y = linear_drive_scale_map["y"] * feedback->pose.position.y;
    wrench_msg.force.y = std::min(wrench_msg.force.y, max_positive_linear_force_map["y"]);
    wrench_msg.force.y = std::max(wrench_msg.force.y, max_negative_linear_force_map["y"]);
  }

  if (linear_drive_scale_map.find("z") != linear_drive_scale_map.end())
  {
    wrench_msg.force.z = linear_drive_scale_map["z"] * feedback->pose.position.z;
    wrench_msg.force.z = std::min(wrench_msg.force.z, max_positive_linear_force_map["z"]);
    wrench_msg.force.z = std::max(wrench_msg.force.z, max_negative_linear_force_map["z"]);
  }

  if (use_stamped_msgs)
  {
    stampAndPublish(wrench_msg);
  }
  else
  {
    wrench_pub->publish(wrench_msg);
  }

  // Make the marker snap back to robot
  server->setPose(robot_name + "_wrench_marker", geometry_msgs::msg::Pose());
  server->applyChanges();
}

void WrenchServerNode::stampAndPublish(geometry_msgs::msg::Wrench& msg) {
  geometry_msgs::msg::WrenchStamped stamped_msg;

  stamped_msg.wrench = msg;
  stamped_msg.header.stamp = this->get_clock()->now();

  wrench_stamped_pub->publish(stamped_msg);
}

} // namespace interactive_marker_wrench_server

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interactive_marker_wrench_server::WrenchServerNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
