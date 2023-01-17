# bellows_ik

This package contains code to control the bellows arm designed and manufactured in the BYU RaD Lab. Manual control is achieved through an xbox controller. The left and right joysticks control the xy and yz planes respectively. The goal position is updated by the xbox controller commands and then converted to desired joint angles through inverse kinematics. A PID controller developed in the RaD lab is used to convert joint angles to pressures for each bellows in the robot.

The package needs to be in the same package as the following packages from the RaD lab (not publicly available):
    - bellows_arm
    - rad_models
    - rad_msgs

The acutal joint angles are recorded and published by the vive motion capture system. Pressure control is run on a beaglebone which is the ROS_MASTER. 
