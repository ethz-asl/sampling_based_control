# From http://wiki.ros.org/roslaunch/API%20Usage
import roslaunch
import time

# import rospy
#
# rospy.init_node('automated_test', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

cli_args = ['mppi_manipulation_royalpanda', 'sim.launch']

number_experiments = 2
time_current_experiment = 0.0
time_per_experiment = 10.0
for experiment in range(number_experiments):
    print("\n\n\n\nStarting new experiment\n\n\n\n")

    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    roslaunch_args = [f'experiment_name:=experiment_{experiment}']
    launch_files = [(roslaunch_file, roslaunch_args)]

    parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    parent.start()
    while time_current_experiment < time_per_experiment:
        time.sleep(10.0)
        print(
            f"Experiment {experiment}: [{time_current_experiment}/{time_per_experiment}]"
        )
        time_current_experiment += 10.0
    time_current_experiment = 0.0
    parent.shutdown()
