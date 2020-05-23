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
	    #parameters = ['/lane_controller/corrected_image/compressed'],
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
	  
    Node(
	    name = 'lanectrl_node',
	    package = 'lane_controller',
	    executable = 'lane_controller',
	    output = 'screen'
	  ),
	  
    Node(
	    name = 'statemachine_node',
	    package = 'statemachine',
	    executable = 'statemachine',
	    output = 'screen'
	  ),
	  
	  
    IncludeLaunchDescription(PythonLaunchDescriptionSource([get_package_share_directory('apriltag_ros'), '/launch/tag_16h5_all.launch.py'])),
  
  
  ])
