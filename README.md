## sampling_based_control


### Installation
1. Clone this repo in the `src` directory of your catkin workspace. 

	```
	~/catkin_ws/src$ git clone --recursive https://github.com/ethz-asl/sampling_based_control.git
	```

2. Install some system dependenices
	```
	sudo apt-get install libyaml-cpp-dev libglfw3-dev`
	```

3. Install ROS dependencies. We manage them with [`rosdep`](http://wiki.ros.org/rosdep). In the root of you catkin workspace do:
	```
	~/catkin_ws$ rosdep init
	~/catkin_ws$ rosdep install --from-paths src/sampling_based_control --ignore-src -y
	```

4. Finally, the manipulation example requires the installation of the `raisim` simulator. Visit its [README](mppi_examples/mppi_manipulation/README.md) and follow the described procedure. 


### Build

For improved performance, make sure to change the build type and then build all packages.
```
~/catkin_ws$ config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
~/catkin_ws$ build
```


### Introdcution

The `mppi_examples` folder contains a set of examples that show how to use the control suite. In general each example needs to implement the following functions and classes:
- _a stage cost_: a function that maps the current state and input to a scalar cost value. You can find a simple implementation of the cost class for a first order mass [here](mppi_examples/mppi_first_order_mass/src/cost.cpp). Cost classes must inherit from the [cost base class](mppi/include/mppi/core/cost.h)
- _system dynamics_: given the currently stored state and applied input predict the next state. You can find a simple implementation of the cost class for a first order mass [here](mppi_examples/mppi_first_order_mass/src/dynamics.cpp). Dynamics classes must inherit from the [cost base class](mppi/include/mppi/core/dynamics.h)
- _controller interface_: finally cost and dynamics can be used in a stochastic controller which uses a gaussian policy to control the system. The [`mppi_ros`](mppi_ros/README.md) provides some utilities classes to simplify the task. Generally, a controller interface istanciate a cost, dynamics (previously defined), a policy (only Gaussian currently supported) and build a [solver](mppi/include/mppi/core/solver.h) class. You can find an example [here](mppi_examples/mppi_first_order_mass/src/controller_interface.cpp#L23-L71). 


In each example, the system dynamics is used also outside the controller as "a simulator" of the system which is controlled. This is a good check, as the model (dynamics in the controller) exactly matches the controlled system (dynamics receiving the input from the controller). See [this](mppi_examples/mppi_first_order_mass/src/nodes/mass_control.cpp#L21-L24) for an example.
