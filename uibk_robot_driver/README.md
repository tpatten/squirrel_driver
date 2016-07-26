<a id="top"/> 
#uibk_robot_driver

**This package is an 8 DOF controller of robotino.

Technical Maintainer: [shangl](https://github.com/shangl/) (Simon Hangl, University of Innsbruck) - simon.hangl@uibk.ac.at

##Contents

1. <a href="#1--installation-requirements">Installation Requirements</a>
2. <a href="#2--execution">Execution</a>
3. <a href="#3--software-architecture">Software architecture</a>


## 1. Installation Requirements: <a id="1--installation-requirements"/> 

Dependencies: geometry_msgs roscpp std_msgs control_toolbox sensor_msgs robotino_driver


## 2. Execution: <a id="2--execution"/> 

Execution:

To run the 8 dof controller, you can use the launch file of the package as follows:


roslaunch uibk_robot_driver controller.launch


The controller publishes information about its current state and configuration on the following topics:


/real/robotino/joint_control/get_state"

/real/robotino/settings/get_command_state

/real/robotino_arm/settings/get_clock_cycle

/real/robotino_arm/joint_control/get_max_dist_per_cycle


After running the controller, it is initialized on freeze mode for safety reasons. To enable movement, a message of type std_msgs/Int32 containing '10' should be sent over:


/real/robotino/settings/switch_mode


In the terminal, you can try:


rostopic pub /real/robotino/settings/switch_mode std_msgs/Int32 "data: 10"



To move the robot, you can either use the move command topic or the ptp command topic as below:


/real/robotino/joint_control/move

/real/robotino/joint_control/ptp


Both expect a message of type std_msgs/Float64MultiArray of length 8, containing absolute target coordinates for the robot's degrees of freedom. The controller relies on the odometry package to determine the robot location and orientation.


The message order is as follows: [orientation, position_x, position_y, arm_joint1_angle, arm_joint1_angle, arm_joint2_angle, arm_joint3_angle, arm_joint4_angle, arm_joint5_angle]


The move command expects to receive a message containing the target states at the frequency of the controller. If no message is received, the robot will stop moving.


On the other hand, the ptp order expects only one message on the ptp topic. After receiving this message, the robot will continue moving to reach the target set values unless another command overrides it. Try avoiding using ptp command because it doesn't have collision avoidance and thus not safe.


If you want to control only some of the controller's degrees of freedom, you can add don't cares in the command array. In the message, “not a number” or nan represents the don't care.






In the terminal, you can try the move command as follows:


rostopic pub /real/robotino/joint_control/move std_msgs/Float64MultiArray "layout:

dim:

- label: ''

size: 8

stride: 8

data_offset: 0

data:

[-0.1, .nan, 0.1, .nan, 0.2, 0.0, .nan, 0.0] "


A demo for trying the controller easily can be launched by:


rosrun uibk_robot_driver uibk_arm_demo






## 3. Software architecture <a id="3--software-architecture"/> 

robot_controller_node: ![robot_controller_node](https://github.com/squirrel-project/squirrel_driver/blob/indigo_dev/uibk_robot_driver/robot_controller_node.png "Architecture")

<a href="#top">top</a>



