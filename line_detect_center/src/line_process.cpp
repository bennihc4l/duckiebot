#include <opencv2/opencv.hpp>


/* -------------------------------------------------------------------------
 * ------------------------------------------------------------------------- */
std::tuple<cv::Point, cv::Point, cv::Mat> line_process(cv::Mat centers, cv::Mat img)
{
	cv::Vec4f regression;
	cv::Point p_regb, p_rege, vect_reg;
	cv::Point p_cross;

	/* process only, if we have at least 2 points */
	if(centers.rows > 1)
	{
		/* calculate regression line */
		cv::fitLine(centers, regression, cv::DIST_L1, 0, 0.01, 0.01);

		/* line with begin, end points. need to be long enough */
		vect_reg = cv::Point(300*regression[0], 300*regression[1]);

		p_regb = cv::Point(regression[2], regression[3]);
		p_rege = p_regb + vect_reg;
		p_regb = p_regb - vect_reg;

		//cv::circle(img, p_regb, 20, cv::Scalar(0,100,0), 3, 8, 0);
		//cv::circle(img, p_rege, 20, cv::Scalar(0,100,0), 3, 8, 0);
		cv::line(img, p_regb, p_rege, cv::Scalar(0,100,0), 2, 8, 0);

	} /* if minimum 2 points */

	return (std::make_tuple(p_regb, p_rege, img));

} /* line_process */
