#ifndef SRC_STATEMACHINE_SRC_SIGN_DETECTOR_HPP_
#define SRC_STATEMACHINE_SRC_SIGN_DETECTOR_HPP_

#include "tf2_msgs/msg/tf_message.hpp"

#define RESULT_CROSS 				1
#define RESULT_TRAFFICLIGHT 2
#define RESULT_STOP				  3

extern void sign_detector(const tf2_msgs::msg::TFMessage::SharedPtr msg);
extern int sign_detector_status();


#endif /* SRC_STATEMACHINE_SRC_SIGN_DETECTOR_HPP_ */
