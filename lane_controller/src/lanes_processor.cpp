#include <opencv2/opencv.hpp>

#include "control_algorithms.hpp"

#define MAX_DEV 200 /* maximum deviation between source and target line */

/* -------------------------------------------------------------------------
 * ------------------------------------------------------------------------- */
std::tuple<cv::Point, cv::Mat>  calc_cross(cv::Point srcb, cv::Point srce, cv::Point tarb, cv::Point tare, cv::Mat img)
{
	cv::Point c;
	cv::Point x = tarb - srcb;
	cv::Point d1 = srce - srcb;
	cv::Point d2 = tare - tarb;

  double cross = d1.x*d2.y - d1.y*d2.x;
  double t1 = (x.x * d2.y - x.y * d2.x)/cross;
  c = srcb + d1 * t1;

  cv::circle(img, c, 10, cv::Scalar(100,0,0), 3, 8, 0);

  return (std::make_tuple(c, img));

} /* calc_cross_center */


/* -------------------------------------------------------------------------
 * ------------------------------------------------------------------------- */
std::tuple<int, cv::Mat> calc_dev_middle(cv::Point cross_c, cv::Point cross_r, cv::Point target_vert, cv::Point target_horz, cv::Mat img)
{
	int lane_err = 0;

	/* mean middle with center and right crossing */
	lane_err = (cross_c.x + cross_r.x) / 2;
	cv::circle(img, cv::Point(lane_err, target_horz.y), 10, cv::Scalar(50, 50, 0), 3, 8, 0);

  /* controller error: right is positive */
	lane_err = lane_err - target_vert.x;

  /* saturate error */
  if(lane_err > MAX_DEV) lane_err = MAX_DEV;
  if(lane_err < -MAX_DEV) lane_err = -MAX_DEV;

  return (std::make_tuple(lane_err, img));

} /* calc_dev_middle */

