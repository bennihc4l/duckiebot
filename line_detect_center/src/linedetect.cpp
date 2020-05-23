#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "lane_controller_msgs/msg/line_detected.hpp"
#include <opencv2/opencv.hpp>

#include "filt_rect_detector.hpp"
#include "line_process.hpp"

#define HORZ_ANGLE 30.0f /* minimum angle to detect horizontal line */

using namespace std::chrono_literals;

rclcpp::Node::SharedPtr g_node = nullptr;
sensor_msgs::msg::CompressedImage newimg;
int roi_top = 233;
int roi_right = 320;
lane_controller_msgs::msg::LineDetected msg_line_right;


/* ---------------------------
 * ---------------------------
 */
static inline std::tuple<lane_controller_msgs::msg::LineDetected, cv::Mat> image_process(cv::Mat img)
{
	cv::Mat img_canny, img_new;
	cv::Mat centers;
	cv::Point p_regb, p_rege;
	lane_controller_msgs::msg::LineDetected msg;

	cv::Rect rect_ROI = cv::Rect(0, roi_top, roi_right, 480-roi_top);

	img_canny = filter_edge_detector(img, rect_ROI);
	std::tie(centers, img) = rect_detector(img_canny, rect_ROI, img);

	std::tie(p_regb, p_rege, img) = line_process(centers, img);

	msg.begin.x = p_regb.x;
	msg.begin.y = p_regb.y;
	msg.end.x = p_rege.x;
	msg.end.y = p_rege.y;

	/* line is valid on minimum number of yellow rects */
	if(centers.rows >= 5)
		msg.valid = true;
	else
		msg.valid= false;

	/* line is horizontal if angle is small enough */
	if(centers.rows > 1)
	{
		double angle = atan( static_cast<double>(p_rege.y - p_regb.y) / static_cast<double>(p_rege.x - p_regb.x) );
		angle = 360/(2*M_PI) * std::fabs(angle);

		cv::putText(img, std::to_string(angle), cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,100,0), 2, 8, false);

		if(angle < HORZ_ANGLE)
			msg.horz = true;
		else
			msg.horz = false;
	}

	cv::rectangle(img, rect_ROI, cv::Scalar(0,0,100), 3, 8, 0);

	return (std::make_tuple(msg, img));
} /* image_process */


/* ---------------------------
 * ---------------------------
 */
void topic_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
	cv::Mat img_input, img_output;
	std::vector<u_char> outbuf;

	img_input = cv::imdecode(msg->data, cv::IMREAD_COLOR);
	std::tie(msg_line_right, img_output) = image_process(img_input);
	cv::imencode(".jpg", img_output, outbuf);

	newimg.data = outbuf;
	newimg.header = msg->header;
	newimg.format = msg->format;
} /* topic callback */


/* ---------------------------
 * ---------------------------
 */
int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::WallRate loop_rate(100ms);
	g_node = rclcpp::Node::make_shared("line_detect");
	auto subscription = g_node->create_subscription<sensor_msgs::msg::CompressedImage>("/None/corrected_image/compressed", 10, topic_callback);

	auto pub_img = g_node->create_publisher<sensor_msgs::msg::CompressedImage>("line_center/corrected_image/compressed", 10);
	auto pub_cam = g_node->create_publisher<sensor_msgs::msg::CameraInfo>("line_center/camera_node/camera_info", 10);
	auto pub_line = g_node->create_publisher<lane_controller_msgs::msg::LineDetected>("line_center/line", 10);

	cv::namedWindow("Parameters_CenterLine", cv::WINDOW_OPENGL);
	cv::createTrackbar("ROI top", "Parameters_CenterLine", &roi_top, 479, 0, 0);
	cv::createTrackbar("ROI right", "Parameters_CenterLine", &roi_right, 639, 0, 0);

	while (rclcpp::ok())
	{

	  cv::waitKey(1);

		pub_img->publish(newimg);
		pub_cam->publish(sensor_msgs::msg::CameraInfo());

		pub_line->publish(msg_line_right);

		rclcpp::spin_some(g_node);
		loop_rate.sleep();

	}

	rclcpp::shutdown();
	subscription = nullptr;
	g_node = nullptr;
	pub_img = nullptr;
	pub_cam = nullptr;
	pub_line = nullptr;

	return (0);
} /* main */
