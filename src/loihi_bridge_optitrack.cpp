// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <iostream>
#include <memory>
#include <utility>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// include optitrack message
#include "custom_interfaces/msg/optitrack_pose.hpp"

// subscriber Optitrack within ROS2
rclcpp::Subscription<custom_interfaces::msg::OptitrackPose>::SharedPtr optitrack_sub_;

// publish optitrack data to ROS1
ros::Publisher ros1_optitrack_pub; 

void optitrackCallback(const custom_interfaces::msg::OptitrackPose::UniquePtr ros2_msg){
  std_msgs::Float64MultiArray ros1_msg;
  
  for(int i=0; i<3; i++){ ros1_msg.data.push_back(ros2_msg->p_w[i]);}
  for(int i=0; i<4; i++){ ros1_msg.data.push_back(ros2_msg->q_wb[i]);}
  
  ros1_optitrack_pub.publish(ros1_msg);
}

int main(int argc, char * argv[])
{
  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("talker");
  optitrack_sub_ = ros2_node->create_subscription<custom_interfaces::msg::OptitrackPose>("optitrack", 10, optitrackCallback);
  

  // ROS 1 node and subscriber
  ros::init(argc, argv, "listener");
  ros::NodeHandle ros1_node;
  ros1_optitrack_pub = ros1_node.advertise<std_msgs::Float64MultiArray>("/ros1_optitrack", 10);

  rclcpp::spin(ros2_node);

  return 0;
}
