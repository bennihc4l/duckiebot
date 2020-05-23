#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>

#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "lane_controller_msgs/msg/lane_controller.hpp"
#include "lane_controller_msgs/msg/line_detected.hpp"

#include "sign_detector.hpp"

using namespace std::chrono_literals;

#define SPEED_BASE 0.3f /* standard travel speed */
#define SPEED_SLOW 0.2f /* speed if travelling without lane follow assistance */

enum Mode {
	MODE_LF,
	MODE_STOP_DETECTED,
	MODE_STANDALONE,
	MODE_STOPPED
};
Mode mode = MODE_LF;

#define DIST_LF_DISABLE  230 /* Distance-Setpoint to red line to disable LF mode */
#define DIST_STOP_THRESH 310 /* Distance-Setpoint to red line to stop at */

rclcpp::Node::SharedPtr g_node = nullptr;

cv::Point line_red[2];
bool line_red_valid = false;


/* ---------------------------
 * ---------------------------
 */
lane_controller_msgs::msg::LaneController statemachine(void)
{
	lane_controller_msgs::msg::LaneController msg;
	int sign_detect = sign_detector_status();

	switch(mode)
	{
		case MODE_LF: {
			if(sign_detect == RESULT_STOP)
			{
				mode = MODE_STOP_DETECTED;
				std::cout << "Stopping on StopSign" << std::endl;
			}
			else
			{
				mode = MODE_LF;
				msg.lf_mode = true;
				msg.speed = SPEED_BASE;
			}

			break;
		} /* lf mode */


		case MODE_STOP_DETECTED: {

			int dist = abs((line_red[1].y + line_red[0].y) / 2);

			/* go straight without lane assist on small distances */
			if(dist > DIST_LF_DISABLE && line_red_valid) {
				mode = MODE_STANDALONE;
			}
			/* go with lane assist if distance is big */
			else if(dist <= DIST_LF_DISABLE || !line_red_valid) {
				std::cout << "Follow w/ LF to straigh line, dist: " << dist << std::endl;
				mode = MODE_STOP_DETECTED;
				msg.lf_mode = true;
				msg.speed = SPEED_BASE;
			}

			break;
		} /* stop detected */

		case MODE_STANDALONE: {

			int dist;

			dist = abs((line_red[1].y + line_red[0].y) / 2);

			/* go without lane assist if distance is small */
			if(dist < DIST_STOP_THRESH || !line_red_valid) {
			std::cout << "Follow w/o LF to straigh line, dist: " << dist << std::endl;
			mode = MODE_STANDALONE;
			msg.lf_mode = false;
			msg.speed = SPEED_SLOW;
			}
			else if(dist >= DIST_STOP_THRESH && line_red_valid) {
				std::cout << "Stopped" << std::endl;
				mode = MODE_STOPPED;
			}

			break;
		} /* standalone mode */

		case MODE_STOPPED: {
			mode = MODE_STOPPED;

			msg.lf_mode = false;
			msg.speed = 0.0f;

			break;
		} /* stopped mode */

	} /* switch(mode) */

	return (msg);

} /* statemachine */


/* ---------------------------
 * ---------------------------
 */
void redline_callback(const lane_controller_msgs::msg::LineDetected::SharedPtr msg)
{
	geometry_msgs::msg::Point geo_b = msg->begin;
	geometry_msgs::msg::Point geo_e = msg->end;
	cv::Point p_srcb = cv::Point(geo_b.x, geo_b.y);
	cv::Point p_srce = cv::Point(geo_e.x, geo_e.y);
	line_red[0] = p_srcb;
	line_red[1] = p_srce;

	line_red_valid = msg->valid;

} /* redline callback */


/* ---------------------------
 * ---------------------------
 */
int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::WallRate loop_rate(100ms);
	g_node = rclcpp::Node::make_shared("statemachine");

	auto subscription_apriltags = g_node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, sign_detector);
	auto subscription_redline = g_node->create_subscription<lane_controller_msgs::msg::LineDetected>("/line_right/line_cross", 10, redline_callback);

	auto pub_lanectrl = g_node->create_publisher<lane_controller_msgs::msg::LaneController>("/lanecontroller/remote", 10);

	lane_controller_msgs::msg::LaneController remote_msg;

	while (rclcpp::ok())
	{
		remote_msg = statemachine();
		pub_lanectrl->publish(remote_msg);
		rclcpp::spin_some(g_node);
		loop_rate.sleep();
	}

	rclcpp::shutdown();
	subscription_apriltags = nullptr;
	subscription_redline = nullptr;
	pub_lanectrl = nullptr;
	g_node = nullptr;

	return (0);
} /* main */
