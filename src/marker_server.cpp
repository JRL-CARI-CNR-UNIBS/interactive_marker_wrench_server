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

  std::vector<rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr> wrench_pub;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_stamped_pub;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server;

  std::map<std::string, double> drive_scale_map;
  std::map<std::string, double> max_positive_force_map;
  std::map<std::string, double> max_negative_force_map;

  bool use_stamped_msgs;

  double marker_size_scale;

  std::vector<std::string> topics;
  std::string link_name;
  std::string robot_name;
  struct Axis {
    constexpr static char LINEAR_X[] = "x";
    constexpr static char LINEAR_Y[] = "y";
    constexpr static char LINEAR_Z[] = "z";
    constexpr static char ANGULAR_X[] = "roll";
    constexpr static char ANGULAR_Y[] = "pitch";
    constexpr static char ANGULAR_Z[] = "yaw";
  };


}; // class WrenchServerNode

WrenchServerNode::WrenchServerNode()
    : rclcpp::Node("wrench_server_node",
                   rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)),
      server(std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "wrench_server", get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
        get_node_topics_interface(), get_node_services_interface())) {
  getParameters();
  for (const auto& topic : topics) {
    if (use_stamped_msgs) {
      wrench_stamped_pub = create_publisher<geometry_msgs::msg::WrenchStamped>("", 1);
    } else {
      wrench_pub.push_back(create_publisher<geometry_msgs::msg::Wrench>(topic, 1));
    }
  }
  createInteractiveMarkers();
  RCLCPP_INFO(get_logger(), "[interactive_marker_wrench_server] Initialized.");
}

void WrenchServerNode::getParameters() {
  rclcpp::Parameter link_name_param;
  rclcpp::Parameter robot_name_param;
  rclcpp::Parameter use_stamped_msgs_param;

  topics = this->get_parameter("topic").as_string_array();

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
  std::map<std::string, double> t_scale;
  if (this->get_parameters("scale", drive_scale_map)) {
    this->get_parameters("max_positive_force", max_positive_force_map);
    this->get_parameters("max_negative_force", max_negative_force_map);
  } else {
    drive_scale_map[Axis::LINEAR_X] = 1.0;
    max_positive_force_map[Axis::LINEAR_X] = 1.0;
    max_negative_force_map[Axis::LINEAR_X] = -1.0;
  }
  marker_size_scale = 0.7;
}

void WrenchServerNode::createInteractiveMarkers() {
  visualization_msgs::msg::InteractiveMarker interactive_marker;
  interactive_marker.header.frame_id = link_name;
  interactive_marker.name = robot_name + "_wrench_marker";
  interactive_marker.description = "wrench controller for " + robot_name;
  interactive_marker.scale = marker_size_scale;

  visualization_msgs::msg::InteractiveMarkerControl control;

  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;

  bool xyz = true;
  if (drive_scale_map.find(Axis::LINEAR_X) != drive_scale_map.end()) {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker.controls.push_back(control);
  } else {
    xyz &= false;
  }

  if (drive_scale_map.find(Axis::LINEAR_Y) != drive_scale_map.end()) {
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker.controls.push_back(control);
  } else {
    xyz &= false;
  }

  if (drive_scale_map.find(Axis::LINEAR_Z) != drive_scale_map.end()) {
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    interactive_marker.controls.push_back(control);
  } else {
    xyz &= false;
  }

  if (xyz) {
    visualization_msgs::msg::InteractiveMarkerControl control_3d;
    control_3d.name = "move_3d";
    control_3d.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
    control_3d.always_visible = true;
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::Type::SPHERE;
    marker.scale.x = interactive_marker.scale * 0.5;
    marker.scale.y = interactive_marker.scale * 0.5;
    marker.scale.z = interactive_marker.scale * 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.95;
    marker.color.b = 0.95;
    marker.color.a = 0.3;
    control_3d.markers.push_back(marker);
    interactive_marker.controls.push_back(control_3d);
  }

  if (drive_scale_map.find(Axis::ANGULAR_X) != drive_scale_map.end()) {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker.controls.push_back(control);
  }

  if (drive_scale_map.find(Axis::ANGULAR_Y) != drive_scale_map.end()) {
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker.controls.push_back(control);
  }

  if (drive_scale_map.find(Axis::ANGULAR_Z) != drive_scale_map.end()) {
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    interactive_marker.controls.push_back(control);
  }

  server->insert(interactive_marker);
  server->setCallback(interactive_marker.name, std::bind(&WrenchServerNode::processFeedback, this, std::placeholders::_1));
  server->applyChanges();
}

void WrenchServerNode::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
  geometry_msgs::msg::Wrench wrench_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);

  if (feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
    // Handle angular change (yaw is the only direction in which you can rotate)
    double yaw, pitch, roll;
    tf2::getEulerYPR(feedback->pose.orientation, yaw, pitch, roll);
    if (drive_scale_map.find(Axis::ANGULAR_X) != drive_scale_map.end()) {
      wrench_msg.torque.x = drive_scale_map[Axis::ANGULAR_X] * roll;
      wrench_msg.torque.x = std::min(wrench_msg.torque.x, max_positive_force_map[Axis::ANGULAR_X]);
      wrench_msg.torque.x = std::max(wrench_msg.torque.x, max_negative_force_map[Axis::ANGULAR_X]);
    }
    if (drive_scale_map.find(Axis::ANGULAR_Y) != drive_scale_map.end()) {
      wrench_msg.torque.y = drive_scale_map[Axis::ANGULAR_X] * pitch;
      wrench_msg.torque.y = std::min(wrench_msg.torque.y, max_positive_force_map[Axis::ANGULAR_Y]);
      wrench_msg.torque.y = std::max(wrench_msg.torque.y, max_negative_force_map[Axis::ANGULAR_Y]);
    }
    if (drive_scale_map.find(Axis::ANGULAR_Z) != drive_scale_map.end()) {
      wrench_msg.torque.z = drive_scale_map[Axis::ANGULAR_X] * yaw;
      wrench_msg.torque.z = std::min(wrench_msg.torque.z, max_positive_force_map[Axis::ANGULAR_Z]);
      wrench_msg.torque.z = std::max(wrench_msg.torque.z, max_negative_force_map[Axis::ANGULAR_Z]);
    }

    if (drive_scale_map.find(Axis::LINEAR_X) != drive_scale_map.end()) {
      wrench_msg.force.x = drive_scale_map[Axis::LINEAR_X] * feedback->pose.position.x;
      wrench_msg.force.x = std::min(wrench_msg.force.x, max_positive_force_map[Axis::LINEAR_X]);
      wrench_msg.force.x = std::max(wrench_msg.force.x, max_negative_force_map[Axis::LINEAR_X]);
    }

    if (drive_scale_map.find(Axis::LINEAR_Y) != drive_scale_map.end()) {
      wrench_msg.force.y = drive_scale_map[Axis::LINEAR_Y] * feedback->pose.position.y;
      wrench_msg.force.y = std::min(wrench_msg.force.y, max_positive_force_map[Axis::LINEAR_Y]);
      wrench_msg.force.y = std::max(wrench_msg.force.y, max_negative_force_map[Axis::LINEAR_Y]);
    }

    if (drive_scale_map.find(Axis::LINEAR_Z) != drive_scale_map.end()) {
      wrench_msg.force.z = drive_scale_map[Axis::LINEAR_Z] * feedback->pose.position.z;
      wrench_msg.force.z = std::min(wrench_msg.force.z, max_positive_force_map[Axis::LINEAR_Z]);
      wrench_msg.force.z = std::max(wrench_msg.force.z, max_negative_force_map[Axis::LINEAR_Z]);
    }
  }

  if (use_stamped_msgs)
  {
    stampAndPublish(wrench_msg);
  }
  else
  {
    for (auto& p : wrench_pub) {
      p->publish(wrench_msg);
    }
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
