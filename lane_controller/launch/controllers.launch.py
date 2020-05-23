import launch

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

  return LaunchDescription([
	  
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
  
  
  ])
