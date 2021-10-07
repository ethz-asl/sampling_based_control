### Install useful dependencies

```
rosdep init
rosdep install --from-paths src --ignore-src -r -y
```

Important: 
- signal_logger
- plotjuggler
- must be with RELEASE build
- raisim -> point to the right version of raisim
- examples


### mppi_examples

Check the `mppi_panda` example for a in depth documentation. 
All other examples follow the same structure with minor per case variations.