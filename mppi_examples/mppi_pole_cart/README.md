### Build

`catkin build mppi_pole_cart`

### Run example

`roslaunch mppi_pole_cart pole_cart_control.launch`


### Data collection
To collect data the bash script located in scripts/data_collection.bash needs to be run.
`bash data_collection.bash`

The script runs the pole_cart_data_collection.launch script and activates state action data collection by passing a save file location. The location of this directory is under `policy_learning/data/` (the data folder must be created before running the script TODO (Kiran): make more general). Under the data directory the script creates another directory uniquely named using the date.
The bash file then loops over `n_runs` of random initial conditions and saves the state action pair data set in a separate file in the above mentioned directory. 
