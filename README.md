# Duckiebot control
ROS2 nodes for gym_duckietown bot controlling

Usage:
Launch files provided in lane_controller package

all.launch.py will launch all necessary packages and let the duckiebot move as defined per statemachine package.

gym_detectors.launch.py launches the duckiebot gym and all detectors without controlling the bot actively.

controllers.launch.py launches active control via lane follow assist and statemachine
