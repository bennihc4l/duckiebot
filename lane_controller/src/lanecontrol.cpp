#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>

#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "duckietown_msgs/msg/wheels_cmd_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "lane_controller_msgs/msg/lane_controller.hpp"
#include "lane_controller_msgs/msg/line_detected.hpp"

#include "control_algorithms.hpp"
#include "lanes_processor.hpp"

using namespace std::chrono_literals;

#define LINE_DIST_CTR    90 /* Distance setpoint between yellow center line and lane center
 	 	 	 	 	 	 	 	 	 	 	 	 	 	   in case of right line loss*/
#define LINE_DIST_RIGHT 450 /* Distance setpoint between right line and lane center
 	 	 	 	 	 	 	 	 	 	 	 	 	 	   in case of center line loss*/

#define FOLLOW_LOSS_THRESH 5 /* Buffer line loss with ticks before turn */

#define SPEED_TURN_FF 0.10 /* feed forward offset speed while turning */
#define SPEED_SLOW 0.25 /* slow speed to circle / drive straight to debounce */

#define KP 0.0005f /* lane controller proportional part */
#define KI 0.0000f /* lane controller integral part */
#define KD 0.01f   /* lane controller differential part */

#define K_PT1 0.34f  /* filter PT1, 0.8 Hz K = Ta / (Ta + 1/(2pi*fgr) ) */

rclcpp::Node::SharedPtr g_node = nullptr;

cv::Point line_center[2], line_right[2];
bool valid_center = false, valid_right = false;

double kp, ki, kd = 0.0f;
int follow_loss_ctr = 0;
bool reset_check_straight = false, reset_check_turn = false;

bool lf_mode = true;
double speed_msg = 0.0f;

cv::Mat img_input, img_output = cv::Mat::zeros(640,480, CV_8UC1);
sensor_msgs::msg::CompressedImage newimg;
sensor_msgs::msg::CompressedImage newimg_data;

T_PID t_pid;
T_PT1 t_pt1;


/* ---------------------------
 * ---------------------------
 */
static std::tuple <double, double, cv::Mat> control(bool lf_mode, double speed_msg, cv::Mat img)
{

	int lane_error, lane_error_filt;
	double speed_left, speed_right, lr_offset;
	double speed_offset = speed_msg;

	cv::Point p_cross_center, p_cross_right;

	/* Target lines */
	cv::Point p_target_vertb = cv::Point(320, 480);
	cv::Point p_target_verte = cv::Point(320,   0);
	cv::Point p_target_horzb = cv::Point(640, 300);
	cv::Point p_target_horze = cv::Point(0,   300);
	cv::line(img, p_target_vertb, p_target_verte, cv::Scalar(0,0,100), 2, 8, 0);
	cv::line(img, p_target_horzb, p_target_horze, cv::Scalar(0,0,100), 2, 8, 0);

	/* Crossings left / right */
	std::tie(p_cross_center, img) = calc_cross(line_center[0], line_center[1], p_target_horzb, p_target_horze, img);
	std::tie(p_cross_right, img) = calc_cross(line_right[0], line_right[1], p_target_horzb, p_target_horze, img);

	/* lane follow center */
	if(valid_center && valid_right && lf_mode)
	{
		cv::putText(img, "Follow", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,100,0), 2, 8, false);

		/* single reset after turn processing */
		if(!reset_check_turn)
		{
			reset_check_turn = true;
			speed_left = 0; speed_right = 0;
			reset_pid_control(&t_pid);
			reset_pt1(&t_pt1);
		}

		follow_loss_ctr = 0;
		reset_check_straight = false;

		/* deviation to lane center */
		std::tie(lane_error, img) = calc_dev_middle(p_cross_center, p_cross_right,  p_target_vertb, p_target_horzb, img);

		lane_error_filt = filter_pt1(lane_error, &t_pt1);
		lr_offset = pid_control(static_cast<double>(lane_error_filt), &t_pid);

		speed_left = speed_offset + lr_offset;
		speed_right = speed_offset - lr_offset;

	} /* if lane follow center */

	/* turn assumed */
	else if(lf_mode)
	{
	  /* min 1 line loss -> buffer with counter */
		if(follow_loss_ctr <= FOLLOW_LOSS_THRESH)
		{
			follow_loss_ctr++;
			/* first drive straight */
			speed_left = SPEED_SLOW;
			speed_right = SPEED_SLOW;
		}

		else
		{
		  cv::putText(img, "Straight Loss", cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,100,0), 2, 8, false);

		  reset_check_turn = false;

			/* single reset after lanefollow processing */
			if(!reset_check_straight)
			{
				reset_check_straight = true;
				speed_left = 0; speed_right = 0;
				reset_pid_control(&t_pid);
				reset_pt1(&t_pt1);
			}

			/* left turn */
			if(!valid_center && valid_right)
			{
				cv::putText(img, "LT", cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,100,0), 2, 8, false);

				/* Deviation between right line and center-middle distance */
				p_target_vertb = cv::Point(LINE_DIST_RIGHT, 480);
				std::tie(lane_error, img) = calc_dev_middle(p_cross_right, p_cross_right, p_target_vertb, p_target_horzb, img);

				lane_error_filt = filter_pt1(lane_error, &t_pt1);
				lr_offset = pid_control(static_cast<double>(lane_error_filt), &t_pid);

				speed_left = speed_offset + lr_offset - SPEED_TURN_FF;
				speed_right = speed_offset - lr_offset + SPEED_TURN_FF;

			} /* if left turn assumed */

				/* right turn */
			if(!valid_right && valid_center)
			{
			  cv::putText(img, "RT", cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,100,0), 2, 8, false);

				/* Deviation between center line and center-middle distance */
				p_target_vertb = cv::Point(LINE_DIST_CTR, 480);
				std::tie(lane_error, img) = calc_dev_middle(p_cross_center, p_cross_center, p_target_vertb, p_target_horzb, img);

				lane_error_filt = filter_pt1(lane_error, &t_pt1);
				lr_offset = pid_control(static_cast<double>(lane_error_filt), &t_pid);

				/* additionally feed-forward control */
				speed_left = speed_offset + lr_offset + SPEED_TURN_FF;
				speed_right = speed_offset - lr_offset - SPEED_TURN_FF;

			} /* if right turn assumed */

				/* If no valid line is detected, circle right */
				if(!valid_right && !valid_center)
				{
					cv::putText(img, "Circling right", cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,100,0), 2, 8, false);
					speed_left = SPEED_SLOW;
					speed_right = 0;
				}

			} /* else not lane follow center */

	} /* else follow loss complete */

	/* non lane-follow mode */
	else if(!lf_mode)
	{
		speed_left = speed_right = speed_msg;
	} /* non lane-follow mode */

	return (std::make_tuple(speed_left, speed_right, img));

} /* control */


/* ---------------------------
 * ---------------------------
 */
void line_center_callback(const lane_controller_msgs::msg::LineDetected::SharedPtr msg)
{

	geometry_msgs::msg::Point geo_b = msg->begin;
	geometry_msgs::msg::Point geo_e = msg->end;
	cv::Point p_srcb = cv::Point(geo_b.x, geo_b.y);
	cv::Point p_srce = cv::Point(geo_e.x, geo_e.y);
	line_center[0] = p_srcb;
	line_center[1] = p_srce;

	valid_center = msg->valid;

	cv::line(img_output, p_srcb, p_srce, cv::Scalar(0,100,0), 2, 8, 0);

} /* topic line_center_callback */


/* ---------------------------
 * ---------------------------
 */
void line_right_callback(const lane_controller_msgs::msg::LineDetected::SharedPtr msg)
{

	geometry_msgs::msg::Point geo_b = msg->begin;
	geometry_msgs::msg::Point geo_e = msg->end;
	cv::Point p_srcb = cv::Point(geo_b.x, geo_b.y);
	cv::Point p_srce = cv::Point(geo_e.x, geo_e.y);
	line_right[0] = p_srcb;
	line_right[1] = p_srce;

	valid_right = msg->valid;

	cv::line(img_output, p_srcb, p_srce, cv::Scalar(0,100,0), 2, 8, 0);

} /* topic line_right_callback */


/* ---------------------------
 * ---------------------------
 */
void remote_callback(const lane_controller_msgs::msg::LaneController::SharedPtr msg)
{
	speed_msg = static_cast<double>(msg->speed);
	lf_mode = static_cast<bool>(msg->lf_mode);

//	t_pid.kp = static_cast<double>(msg->kp);
//	t_pid.ki = static_cast<double>(msg->ki);
//	t_pid.kd = static_cast<double>(msg->kd);

} /* topic remote_callback */

/* ---------------------------
 * ---------------------------
 */
void img_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{

	std_msgs::msg::Header img_header;
	std_msgs::msg::String img_format;

	img_input = cv::imdecode(msg->data, cv::IMREAD_COLOR);

	newimg_data.header = msg->header;
	newimg_data.format = msg->format;

} /* topic img_callback */


/* ---------------------------
 * ---------------------------
 */
int main(int argc, char * argv[])
{

	reset_pid_control(&t_pid);
	reset_pt1(&t_pt1);

	t_pid.kp = KP;
	t_pid.ki = KI;
	t_pid.kd = KD;

	t_pt1.k = K_PT1;

	rclcpp::init(argc, argv);
	rclcpp::WallRate loop_rate(100ms);
	g_node = rclcpp::Node::make_shared("lane_controller");

	auto subscription_line_center = g_node->create_subscription<lane_controller_msgs::msg::LineDetected>("/line_center/line", 10, line_center_callback);
	auto subscription_line_right = g_node->create_subscription<lane_controller_msgs::msg::LineDetected>("/line_right/line_right", 10, line_right_callback);

	auto subscription_remote = g_node->create_subscription<lane_controller_msgs::msg::LaneController>("/lanecontroller/remote", 10, remote_callback);
	auto subscription_img = g_node->create_subscription<sensor_msgs::msg::CompressedImage>("/None/corrected_image/compressed", 10, img_callback);

	auto pub_wheels = g_node->create_publisher<duckietown_msgs::msg::WheelsCmdStamped>("None/wheels_driver_node/wheels_cmd", 10);
	auto pub_img = g_node->create_publisher<sensor_msgs::msg::CompressedImage>("lanecontroller/corrected_image/compressed", 10);
	auto pub_cam = g_node->create_publisher<sensor_msgs::msg::CameraInfo>("lanecontroller/camera_node/camera_info", 10);

	while (rclcpp::ok())
	{

		double speed_left, speed_right;

		std::tie(speed_left, speed_right, img_output) = control(lf_mode, speed_msg, img_output);

		duckietown_msgs::msg::WheelsCmdStamped msg_wheels;
		msg_wheels.vel_left = speed_left;
		msg_wheels.vel_right = speed_right;
		pub_wheels->publish(msg_wheels);

		/* output image */
		std::vector<u_char> outbuf;
	  if(!img_output.empty())
			cv::imencode(".jpg", img_output, outbuf);

		newimg.data = outbuf;
		newimg.header = newimg_data.header;
		newimg.format = newimg_data.format;
		pub_img->publish(newimg);
		pub_cam->publish(sensor_msgs::msg::CameraInfo());

		if(!img_input.empty())
			img_input.copyTo(img_output);


		rclcpp::spin_some(g_node);
		loop_rate.sleep();
	}

	rclcpp::shutdown();
	subscription_line_center = nullptr;
	subscription_line_right = nullptr;
	subscription_remote = nullptr;
	pub_wheels = nullptr;
	pub_img = nullptr;
	pub_cam = nullptr;
	g_node = nullptr;

	return (0);
} /* main */
