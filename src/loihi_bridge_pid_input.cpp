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
#include "std_msgs/Int32MultiArray.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

// subscriber input within ROS2
rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr input_sub_;

// publish input data to ROS1
ros::Publisher input_pub; 

void inputCallback(const std_msgs::msg::Int32MultiArray::UniquePtr ros2_msg){
  std_msgs::Int32MultiArray ros1_msg;
  for(int i=0; i<3; i++){ ros1_msg.data.push_back(ros2_msg->data[i]);}
  input_pub.publish(ros1_msg);
}

int main(int argc, char * argv[])
{
  // ROS 2 node and publisher
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("talker");
  input_sub_ = ros2_node->create_subscription<std_msgs::msg::Int32MultiArray>("loihi_input", 10, inputCallback);
  

  // ROS 1 node and subscriber
  ros::init(argc, argv, "listener");
  ros::NodeHandle ros1_node;
  input_pub = ros1_node.advertise<std_msgs::Int32MultiArray>("/loihi_input", 10);

  rclcpp::spin(ros2_node);

  return 0;
}
