## sampling_based_control

### Dependencies
- [`pinocchio`](https://stack-of-tasks.github.io/pinocchio/download.html)
    
### Build

- `catkin build mppi_manipulation -DCMAKE_BUILD_TYPE=RelWithDebInfo`

### Run

Terminal #1
`roslaunch mppi_manipulation control.launch fixed_base:=false`

Terminal #2
`rosrun mppi_manipulation demo.py`


