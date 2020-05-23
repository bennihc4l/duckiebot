#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "lane_controller_msgs/msg/line_detected.hpp"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

rclcpp::Node::SharedPtr g_node = nullptr;
sensor_msgs::msg::CompressedImage newimg;
cv::Mat img_input, img_output;

lane_controller_msgs::msg::LineDetected msg_line_right;
lane_controller_msgs::msg::LineDetected msg_line_cross;

int rightline_thres = 80;
int rightline_minangl = 80;
int rightline_maxangl = 85;
int red_roi_top = 200;
int red_roi_left = 50;
int red_roi_height = 80;
int red_roi_width = 150;
int red_roi_minangl = 70;
int red_roi_maxangl = 130;

/* ---------------------------
 * ---------------------------
 */
static inline cv::Mat image_process(cv::Mat img)
{
	cv::Mat img_gauss, img_hsv, img_thresh, img_canny;
	std::vector<std::vector<cv::Point> > contours;

	cv::GaussianBlur(img, img_gauss, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);

	cv::cvtColor(img_gauss, img_hsv, cv::COLOR_BGR2HSV, 0);

	cv::inRange(img_hsv,
							cv::Scalar(  0,  0, 100),
							cv::Scalar(255, 10, 255),
							img_thresh);

	cv::Canny(img_thresh, img_canny, 1, 1, 5, false);


	/* Right line for follower */
  std::vector<cv::Vec2f> lines_right;
  cv::Vec2f line_single;
  cv::HoughLines(img_canny, lines_right, 1, M_PI/72, rightline_thres, 0, 0,
  							(static_cast<double>(rightline_minangl) / 100.0) *3*M_PI/4,
								(static_cast<double>(rightline_maxangl) / 100.0) *M_PI);

  /* valid if min 1 line exists */
  if(lines_right.size() > 0)
  {
  	line_single = lines_right[0];
  	msg_line_right.valid = true;
  }
  else
  {
  	msg_line_right.valid = false;
  }

  /* point with largest theta is inner line */
  if(lines_right.size() > 1)
  {
  	line_single = lines_right[0];
		float theta_largest = 0.0f;

		for(size_t i = 0; i < lines_right.size()-1; i++)
		{
			float theta_act = lines_right[i][1];

			if(theta_act > theta_largest)
			{
				theta_largest = theta_act;
				line_single = lines_right[i];
			}
		} /* for all lines */
  } /* if size > 1 */

  float rho = line_single[0], theta = line_single[1];
  cv::Point pt1, pt2;
  double a = cos(theta), b = sin(theta);
  double x0 = a*rho, y0 = b*rho;
  pt1.x = std::round(x0 + 1000*(-b));
  pt1.y = std::round(y0 + 1000*(a));
  pt2.x = std::round(x0 - 1000*(-b));
  pt2.y = std::round(y0 - 1000*(a));
  cv::line(img, pt1, pt2, cv::Scalar(0,0,255), 3, cv::LINE_AA);

  msg_line_right.begin.x = pt1.x;
  msg_line_right.begin.y = pt1.y;
  msg_line_right.end.x = pt2.x;
  msg_line_right.end.y = pt2.y;

  /* -------------------------------------------------------------------- */
  /* Horizontal white line for left turn */

  cv::Rect roi_horz_ctr = cv::Rect(20, 173, 530, 60);
  std::vector<cv::Vec2f> lines_horz;

//  cv::rectangle(img, roi_horz_ctr, cv::Scalar(0,100,0), 3, 8, 0);
  cv::HoughLines(img_canny(roi_horz_ctr), lines_horz, 1, M_PI/72, 80, 0, 0, 0.9*M_PI/2, 1.1*M_PI/2);

  /* simplify - take first line in vector */
  if(lines_horz.size() > 0)
  {
		float rho = lines_horz[0][0], theta = lines_horz[0][1];
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = std::round(x0 + 1000*(-b)) + roi_horz_ctr.x;
		pt1.y = std::round(y0 + 1000*(a)) + roi_horz_ctr.y;
		pt2.x = std::round(x0 - 1000*(-b)) + roi_horz_ctr.x;
		pt2.y = std::round(y0 - 1000*(a)) + roi_horz_ctr.y;
		cv::line(img, pt1, pt2, cv::Scalar(0,255,0), 3, cv::LINE_AA);

		msg_line_right.horz = true;
  }
  else
  {
  	msg_line_right.horz = false;
  }

  /* -------------------------------------------------------------------- */
  /* Horizontal red line for crossing detect */
	cv::Mat img_thresh_red, img_canny_red;

  cv::inRange(img_hsv,
							cv::Scalar(175,  10,  50),
							cv::Scalar(180, 255, 255),
							img_thresh_red);

	cv::Canny(img_thresh_red, img_canny_red, 1, 1, 5, false);


	/* Red line for crossing */
	cv::Rect roi_horz_red = cv::Rect(179, 132, 257, 325);
	cv::rectangle(img, roi_horz_red, cv::Scalar(0,100,0), 3, 8, 0);

  std::vector<cv::Vec2f> lines_red;
  cv::HoughLines(img_canny_red(roi_horz_red), lines_red, 1, M_PI/72, 80, 0, 0, (static_cast<double>(red_roi_minangl)/100)*M_PI/2,
  																																						 (static_cast<double>(red_roi_maxangl)/100)*M_PI/2 );

  /* simplify - take first line in vector */
  if(lines_red.size() > 0)
  {
		float rho = lines_red[0][0], theta = lines_red[0][1];
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = std::round(x0 + 1000*(-b)) + roi_horz_red.x;
		pt1.y = std::round(y0 + 1000*(a)) + roi_horz_red.y;
		pt2.x = std::round(x0 - 1000*(-b)) + roi_horz_red.x;
		pt2.y = std::round(y0 - 1000*(a)) + roi_horz_red.y;
		cv::line(img, pt1, pt2, cv::Scalar(100,0,0), 3, cv::LINE_AA);

		msg_line_cross.begin.x = pt1.x;
		msg_line_cross.begin.y = pt1.y;
		msg_line_cross.end.x = pt2.x;
		msg_line_cross.end.y = pt2.y;
		msg_line_cross.valid = true;

  } /* if lines_red.size > 0 */
  else
  {
  	msg_line_cross.valid = false;
  }


  return (img);

} /* image_process */


/* ---------------------------
 * ---------------------------
 */
void topic_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
	std::vector<u_char> outbuf;

	img_input = cv::imdecode(msg->data, cv::IMREAD_COLOR);
	img_output = image_process(img_input);
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

	auto pub_img = g_node->create_publisher<sensor_msgs::msg::CompressedImage>("line_right/corrected_image/compressed", 10);
	auto pub_cam = g_node->create_publisher<sensor_msgs::msg::CameraInfo>("line_right/camera_node/camera_info", 10);

	auto pub_line_right = g_node->create_publisher<lane_controller_msgs::msg::LineDetected>("line_right/line_right", 10);
	auto pub_line_cross = g_node->create_publisher<lane_controller_msgs::msg::LineDetected>("line_right/line_cross", 10);

	cv::namedWindow("Parameters_RightLine", cv::WINDOW_OPENGL);
	cv::createTrackbar("Right hough threshold", "Parameters_RightLine", &rightline_thres, 400, 0, 0);
	cv::createTrackbar("Right hough min angle", "Parameters_RightLine", &rightline_minangl, 200, 0, 0);
	cv::createTrackbar("Right hough max angle", "Parameters_RightLine", &rightline_maxangl, 100, 0, 0);
	cv::createTrackbar("Red ROI Top", "Parameters_RightLine", &red_roi_top, 480, 0, 0);
	cv::createTrackbar("Red ROI Left", "Parameters_RightLine", &red_roi_left, 640, 0, 0);
	cv::createTrackbar("Red ROI Height", "Parameters_RightLine", &red_roi_height, 480, 0, 0);
	cv::createTrackbar("Red ROI Width", "Parameters_RightLine", &red_roi_width, 640, 0, 0);
	cv::createTrackbar("Red line min angle", "Parameters_RightLine", &red_roi_minangl, 100, 0, 0);
	cv::createTrackbar("Red line max angle", "Parameters_RightLine", &red_roi_maxangl, 200, 0, 0);


	while (rclcpp::ok())
	{
		cv::waitKey(1);

		pub_img->publish(newimg);
		pub_cam->publish(sensor_msgs::msg::CameraInfo());

		pub_line_right->publish(msg_line_right);
		pub_line_cross->publish(msg_line_cross);

		rclcpp::spin_some(g_node);
		loop_rate.sleep();

	}

	rclcpp::shutdown();
	subscription = nullptr;
	g_node = nullptr;
	pub_img = nullptr;
	pub_cam = nullptr;

	pub_line_right = nullptr;
	pub_line_cross = nullptr;

	return (0);
} /* main */
