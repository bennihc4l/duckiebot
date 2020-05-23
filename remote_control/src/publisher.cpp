// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "duckietown_msgs/msg/wheels_cmd_stamped.hpp"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */

int main(int argc, char * argv[])
{
	int cmd = 0;

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("line_detect");
  auto publisher = node->create_publisher<duckietown_msgs::msg::WheelsCmdStamped>("None/wheels_driver_node/wheels_cmd", 10);
  duckietown_msgs::msg::WheelsCmdStamped message;
  rclcpp::WallRate loop_rate(100ms);
  
	cv::namedWindow("RC", cv::WINDOW_OPENGL);
	cv::resizeWindow("testwin", 10, 10);
  
  while (rclcpp::ok()) {

  	cmd = cv::waitKey(10);

    if(cmd == 119)
    {
        message.vel_left = 0.5;
        message.vel_right = 0.5;
    }
    else if(cmd == 97)
    {
        message.vel_left = -0.5;
        message.vel_right = 0.5;
    }
    else if(cmd == 115)
    {
        message.vel_left = -0.5;
        message.vel_right = -0.5;
    }
    else if(cmd == 100)
    {
        message.vel_left = 0.5;
        message.vel_right = -0.5;
    }
    else if(cmd == -1)
    {
        message.vel_left = 0.0;
        message.vel_right = 0.0;
    }
    
    
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();

  return (0);
}
