## Install Instructions

Create a ROS workspace and install the following dependencies. 
### Dependencies
- Install the `pinocchio` library following the instructions from the [ufficial documentation](https://stack-of-tasks.github.io/pinocchio/download.html). This is the Rigid Body Dynamics (RBD) modeling library we use. 
- Clone and install the _Savitzky-Golay filetr_ **from source**, following the instructions in [the repo](https://github.com/arntanguy/gram_savitzky_golay).
- Clone the following repos in the src folder of your catkin workspace:
    - Dependency of `any_worker`: `git clone git@github.com:ANYbotics/message_logger.git`
    - Required for multi-threading: `git clone git@github.com:ANYbotics/any_node.git`
    - This repo: `git clone git@github.com:ethz-asl/sampling_based_control.git`
- Install additional dependencies script which is in the `sampling_based_control` repo:  `sudo ./install_dependencies.sh`
    
## Build

- Configure the workspace to compile in Release mode: `catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo`

- Build the stack **and** the examples: `catkin build mppi_control`. This will build three examples: 
    - `mppi_panda`: controller for the 7-DOF arm _panda_ with a fixed base
    - `mppi_panda_mobile`: controller for 7-DOF arm _panda_ with a omni-directional mobile base
    - `mppi_pole_cart`: controller for the pole-cart example

## Run the examples

- `roslaunch mppi_panda panda_control.launch`
- `roslaunch mppi_panda_mobile panda_mobile_control.launch`
- `roslaunch mppi_pole_cart pole_cart_control.launch`

### References
- [Information Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving](https://arxiv.org/abs/1707.02342), _Willias at al._


