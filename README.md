## sampling_based_control


### Install instructions
Clone this repo in the `src` directory of your catkin workspace. Install all dependencies first and then build the desired packages. 

### System dependencies
- `yamlcpp`: `$ sudo apt-get install libyaml-cpp-dev`

### ROS dependencies
- `<catkin_workspace>$ rosdep install --from-paths src/sampling_based_control --ignore-src -y`  
- [`rqt_multiplot`](https://github.com/anybotics/rqt_multiplot_plugin) (Optional)
    
### Build

`catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`

Kinematic planning

This package uses a kinematic model to track the end effector of a mobile manipulator. The base can be made either _holonomic_ or _non-holonomic_.

Build the package with `catkin build mppi_panda_mobile`

Manipulation control

The `mppi_manipulation` package requires the `raisim` physical simulator. This can be installed following the instructions reported [here](https://raisim.com/sections/Installation.html). As raisim examples and python pindings are not needed you can disable them setting `DRAISIM_EXAMPLE=OFF` and  `-DRAISIM_PY=OFF`. Note that an academic licence can be obtained contacting the developers. Make sure to add the following lines to your `~/.bashrc` in order to find `raisim` when building with catkin:

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<where-you-installed-raisim>/lib
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:<where-you-installed-raisim>
```

Build the package with `catkin build mppi_manipulation`

### Run the manipulation demo

Terminal #1

`roslaunch mppi_manipulation control.launch fixed_base:=false`

Terminal #2

`rosrun mppi_manipulation demo.py`
