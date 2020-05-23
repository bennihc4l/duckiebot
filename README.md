# Duckiebot control
ROS2 nodes for gym_duckietown bot controlling
===

** 1. General Usage: **
------------------

Launch files provided in lane_controller package

* all.launch.py will launch all necessary packages and let the duckiebot move as defined per statemachine package.

* gym_detectors.launch.py launches the duckiebot gym and all detectors without controlling the bot actively.

* controllers.launch.py launches active control via lane follow assist and statemachine

** 2. Remote control: **
------------------

Package remote_control provides an executable, which controls the _wheels_ command directly via an QT window interface.
Pressing the key send the command, releasing it will stop the movement.
* _W_: Straight forward
* _A_: Left turn
* _S_: Straight backward
* _D_: Right turn
* _E_: Stop any movement
