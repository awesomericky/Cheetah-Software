## Concurrent Training of a Control Policy and a State Estimator for Dynamic and Robust Legged Locomotion
This repository is for sharing our implementation code on the Mini cheetah.
This software is based on the work of MIT-biomimetics Lab.
For the instruction about build, use of simulation, and deployment on the robot, please visit the original repository (https://github.com/mit-biomimetics/Cheetah-Software).

We inserted our controller, RLJointPD, inside the MIT_Controller.

To run our controller in simulation, first change the vairable 'isMinicheetah' to false.
'isMinicheetah' is located in 'user/MIT_Controller/FSM_States/FSM_State_RLJointPD.cpp'.
Then, follow the below code to run mit_ctrl.
```
cd build
make -j4
./user/MIT_Controller/mit_ctrl m s
```
After running mit_ctrl, change 'use_rc' to 0 and 'control_mode' to 53 in SimControlPanel.


To run our controller on the real robot, change the variable 'isMinicheetah' to true.
If 'isMinicheetah' is set to be false, then the real robot cannot walk well because of wrong PD gains.

Concerning the RC controller, we removed all the other locomotion mode and replaced them with our controller.
By choosing the 'run' mode on the RC controller, RLJointPD mode is executed.
