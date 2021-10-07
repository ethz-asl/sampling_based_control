# mppi_first_order_mass

### Build

`catkin build mppi_first_order_mass`

### Run example

The example shows a velocity controlled point mass. First launch the simulation and 
in a second terminal set the goal for the point mass.

A `plotjuggler` GUI should show the mass motion. Make sure that at startup you have all the topic
subscribed by the GUI.

```
# roslaunch mppi_first_oder_mass mass_control.launch`
# rostopic pub -1 /goal geometry_msgs/Point "{x: 1.0, y: 1.0, z: 0.0}"
```

### Show control GUI

To show the control gui for the previous demo, launch `model_tracking.launch`
