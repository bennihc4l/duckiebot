import launch

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

  return LaunchDescription([
	
    Node(
	      name = 'gym_node',
	      package = 'gym_duckietown_ros2_agent',
	      executable = 'rosagent',
	      output = 'screen'
    ),
	
    Node(
	    name = 'view_node',
	    package = 'rqt_image_view',
	    executable = 'rqt_image_view',
	    #parameters = ['/None/corrected_image/compressed'],
	    output = 'screen'
	  ),
	  
    Node(
	    name = 'line_center_node',
	    package = 'line_detect_center',
	    executable = 'line_detect_center',
	    output = 'screen'
	  ),
	  
    Node(
	    name = 'line_right_node',
	    package = 'line_detect_right',
	    executable = 'line_detect_right',
	    output = 'screen'
      ),
	  
	  
    IncludeLaunchDescription(PythonLaunchDescriptionSource([get_package_share_directory('apriltag_ros'), '/launch/tag_16h5_all.launch.py'])),
  
  
  ])
