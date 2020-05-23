#include <opencv2/opencv.hpp>

#include "filt_rect_detector.hpp"


/* -------------------------------------------------------------------------
 * ------------------------------------------------------------------------- */
cv::Mat filter_edge_detector(cv::Mat img, cv::Rect roi)
{

	/* ROI is lower left corner */
	img = img(roi);

	cv::Mat img_hsv, img_thresh, img_canny;

	/* find yellow components in HSV colorspace */
	cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV, 0);
	cv::inRange(img_hsv,
							cv::Scalar(24, 59, 110),
							cv::Scalar(88, 255, 207),
							img_thresh);

	/* filter with canny algorithm */
	cv::Canny(img_thresh, img_canny, 1, 1, 5, false);

	return (img_canny);

} /* img_filter */


/* -------------------------------------------------------------------------
 * ------------------------------------------------------------------------- */

std::tuple<cv::Mat, cv::Mat> rect_detector(cv::Mat img_filt, cv::Rect roi, cv::Mat img_orig)
{

	std::vector<std::vector<cv::Point> > contours;
	cv::Mat center_points;
	cv::Mat img_polygons;

	/* find contours */
	cv::findContours(img_filt, contours, cv::RETR_LIST, cv::CHAIN_APPROX_TC89_L1, cv::Point(0,0));

	/* loop over all contours */
	if(contours.size() > 0)
	{
		for(uint i=0; i<=contours.size()-1; i++)
		{
//			cv::drawContours(img_orig, contours, i, cv::Scalar(0,100,0), 3, 8);

			/* filter out small arc lengths */
			double arclen = cv::arcLength(contours[i], true);
			if(arclen >= 55)
			{
				/* find polygon within contours */
				cv::approxPolyDP(contours[i], img_polygons, 0.005*arclen, true);
				cv::Rect rect = cv::boundingRect(img_polygons);
				/* remap ROI->base coordinates */
				rect.x += roi.x;
				rect.y += roi.y;

				cv::rectangle(img_orig, rect, cv::Scalar(100,0,0));

				/* find point in center of each rectangle */
				cv::Point center_p;
				center_p = cv::Point(rect.x + rect.width*0.5, rect.y + rect.height*0.5);
				center_points.push_back(center_p);
				cv::circle(img_orig, center_p, 5, cv::Scalar(0,100,0), 5, 8, 0);

			} /* if arclen */
		} /* for */
	} /* if number contours > 0 */

	return (std::make_tuple(center_points, img_orig));

} /* find_rectangles */

