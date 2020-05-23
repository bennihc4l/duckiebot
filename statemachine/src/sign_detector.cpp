#include "rclcpp/rclcpp.hpp"
#include "sign_detector.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#define TAG_CROSS         8
#define TAG_TRAFFICLIGHT 74
#define TAG_STOP				  1

static int detected_sign = 0;

/* ---------------------------
 * ---------------------------
 */
void sign_detector(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
	geometry_msgs::msg::TransformStamped tfStamped;
	std::string str;
	std::vector<std::string> strvect;
	int nr = 0;

	if(msg->transforms.size() > 0)
	{
		for(uint i=0; i < msg->transforms.size(); i++)
		{
			tfStamped = msg->transforms[i];
			str = tfStamped.child_frame_id;

			/* sign number is first element after ":" */
			boost::split(strvect, str, boost::is_any_of(":"));
			nr = std::stoi(strvect[1], nullptr, 0);

//	  double x = tfStamped.transform.translation.x;
//	  double y = tfStamped.transform.translation.y;
//		double z = tfStamped.transform.translation.z;

			switch(nr)
			{
				case TAG_CROSS: {
//				  std::cout << "Crossing Detected" << std::endl;
				  detected_sign = RESULT_CROSS;
				  break;
				}
				case TAG_TRAFFICLIGHT: {
//				  std::cout << "Trafficlight Detected" << std::endl;
				  detected_sign = RESULT_TRAFFICLIGHT;
				  break;
				}
				case TAG_STOP: {
//				  std::cout << "StopSign Detected" << std::endl;
				  detected_sign = RESULT_STOP;
				  break;
				}
				default: {
					detected_sign = 0;
					break;
				}
			} /* switch apriltag nr */

//			std::cout << "Nr: " << i << " type: " << strvect[1] << std::endl;

		} /* for over all detected signs */
	} /* if at least one message */

} /* sign_detector */


/* ---------------------------
 * ---------------------------
 */
int sign_detector_status()
{
	return (detected_sign);
} /* sign_detector_status */

