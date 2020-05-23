#ifndef SRC_LINE_DETECT_SRC_FILT_RECT_DETECTOR_HPP_
#define SRC_LINE_DETECT_SRC_FILT_RECT_DETECTOR_HPP_

extern cv::Mat filter_edge_detector(cv::Mat img, cv::Rect roi);
extern std::tuple<cv::Mat, cv::Mat> rect_detector(cv::Mat img_filt, cv::Rect roi, cv::Mat img_orig);


#endif /* SRC_LINE_DETECT_SRC_FILT_RECT_DETECTOR_HPP_ */
