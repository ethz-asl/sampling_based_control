## sampling_based_control


### Install instructions
Clone this repo in the `src` directory of your catkin workspace. Install all dependencies first and then build the desired packages. We manage rosdependecies using [`rosdep`](http://wiki.ros.org/rosdep). 

### System dependencies
```
sudo apt-get install libyaml-cpp-dev libglfw3-dev`
```
    
### Build

- Init rosdep and install all packages dependencies
	```
	rosdep init
	rosdep install --from-paths src/sampling_based_control --ignore-src -y
	```

- Before building all packages, the manipulation example requires the installation of the `raisim` simulator. Visit its [README](mppi_examples/mppi_manipulation/README.md) and follow the described procedure. 

- Finally, configure the catkin workspace and build all packages
	```
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
	catkin build
	```
