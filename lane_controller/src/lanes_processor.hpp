#ifndef SRC_LANE_CONTROLLER_SRC_LANES_PROCESSOR_HPP_
#define SRC_LANE_CONTROLLER_SRC_LANES_PROCESSOR_HPP_


extern std::tuple<cv::Point, cv::Mat>  calc_cross(cv::Point srcb, cv::Point srce, cv::Point tarb, cv::Point tare, cv::Mat img);
extern std::tuple<int, cv::Mat> calc_dev_middle(cv::Point cross_c, cv::Point cross_r, cv::Point target_vert, cv::Point target_horz, cv::Mat img);


#endif /* SRC_LANE_CONTROLLER_SRC_LANES_PROCESSOR_HPP_ */
