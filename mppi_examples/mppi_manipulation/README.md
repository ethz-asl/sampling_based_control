### mppi_manipulation

This package requires the installation of `raisim`, an efficient physical simulator. To install this dependency, visit [its website](https://raisim.com/sections/Introduction.html). Running the code also requires an activation key which can be obtained for academic purposes. The license can be requested using the form available at [raisim.com](raisim.com).

Follow the install instruction and place your activation key under `~/.raisim/activation.raisim`. This is the default location. 

### Build

```
catkin build mppi_manipulation
``` 

### Run

In the demo the mobile robot is controlled to open a door. After launching the reference is set in a automated way according to the timestamps and reference vector parse from the [experiments.yaml](config/experiments/references.yaml) file.

```
roslaunch mppi_manipulation control.launch
```
