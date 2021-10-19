# From http://wiki.ros.org/roslaunch/API%20Usage
import roslaunch
import rosnode
import time
import string


def extend_name(experiment_id, object_type, filter_in, filter_out):
    suffix = ""
    if filter_in and filter_out:
        suffix = "_filter_in_out"
    elif filter_in:
        suffix = "_filter_in"
    elif filter_out:
        suffix = "_filter_out"
    return f"{string.ascii_lowercase[experiment_id]}_{object_type}{suffix}"


uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

cli_args = ['mppi_manipulation_royalpanda', 'sim.launch']

objects_op = ["new_shelf"]  #["door", "shelf", "microwave", "drawer"]
filter_in_op = [False, True]
filter_out_op = [False, True]
number_experiments = 5

max_experiment_time = 140

for fin, fout in zip(filter_in_op, filter_out_op):
    for object_type in objects_op:
        for ex_id in range(number_experiments):

            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
                cli_args)[0]

            # Conver boolean to string launch arguments
            fin_arg = "true" if fin else "false"
            fout_arg = "true" if fout else "false"

            roslaunch_args = [
                f"experiment_name:={extend_name(ex_id, object_type, fin, fout)}",
                f'object:={object_type}', f'rollouts:=40',
                f'filter_in:={fin_arg}', f'filter_out:={fout_arg}',
                f'log_folder:=/media/giuseppe/My Passport/Work/logs_mppi/',
                'rviz:=false'
            ]

            launch_files = [(roslaunch_file, roslaunch_args)]

            parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
            parent.start()
            experiment_time = time.time()

            # make sure the simulation node is running
            time.sleep(10.0)

            # while the node is running
            while True:
                nodes = rosnode.get_node_names()
                if "/simulation_node" not in nodes:
                    print(f"Failed to find simulation_node in {nodes}")
                    break

                if (time.time() - experiment_time) > max_experiment_time:
                    print(
                        f"Experiment longer than maximum duration: {time.time() - experiment_time} s."
                    )
                    break

                time.sleep(1.0)

            parent.shutdown()
