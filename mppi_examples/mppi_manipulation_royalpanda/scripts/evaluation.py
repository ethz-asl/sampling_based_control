import os
import numpy as np
import signal_logger
import matplotlib.pyplot as plt

plt.rcParams['axes.grid'] = True
import rospkg

from utilities import *

rospack = rospkg.RosPack()
ROOT_DIR = os.path.join(rospack.get_path("mppi_manipulation_royalpanda"),
                        "logs")
LOG_PREFIX = "mppi_manipulation"
LOG_FILE = os.path.join(ROOT_DIR, "filter.silo")
LOG_FILES = get_files(ROOT_DIR, LOG_PREFIX)
REQUIRED_FIELDS = [
    "log/sim_time", "log/stage_cost", "log/torque_command",
    "log/cartesian_limits_violation", "log/joint_limits_violation",
    "log/solver/rollouts/min_cost", "log/velocity_command",
    "log/velocity_filtered", "log/velocity_measured", "log/position_desired"
]
LOG_FILES_PRINT = '\n'.join(LOG_FILES)
print(f"""
>> Root dir: 
{ROOT_DIR}
>> Prefix: 
{LOG_PREFIX}
>> Log files: 
{LOG_FILES_PRINT}
""")

###############################
# Main
###############################

if __name__ == "__main__":
    for file in LOG_FILES:
        experiment_name = file.split('/')[-1].replace('.silo', '')
        silo = signal_logger.Silo(file)
        print_keys(silo)
        silo_dict = to_dictionary(silo, REQUIRED_FIELDS)

        time = silo_dict['sim_time']
        print(
            f"Min time in experiment={min(time)}, max time in experiment={max(time)}"
        )

        scalar_plot(time,
                    silo_dict['stage_cost'],
                    prefix=f"{experiment_name}_stage_cost")
        scalar_plot(time,
                    silo_dict['min_cost'],
                    prefix=f"{experiment_name}_min_cost")
        matrix_plot(time,
                    silo_dict['position_desired'],
                    prefix=f"{experiment_name}_position_desired")
        matrix_plot(time,
                    silo_dict['velocity_measured'],
                    prefix=f"{experiment_name}_velocity_measured")
        matrix_plot(time,
                    silo_dict['velocity_filtered'],
                    prefix=f"{experiment_name}_velocity_filtered")
        matrix_plot(time,
                    silo_dict['velocity_command'],
                    prefix=f"{experiment_name}_velocity_command")
        matrix_plot(time,
                    silo_dict['torque_command'],
                    prefix=f"{experiment_name}_torque_command")
        matrix_plot(time,
                    silo_dict['joint_limits_violation'],
                    prefix=f"{experiment_name}_joint_limits_violation")
        matrix_plot(time,
                    silo_dict['cartesian_limits_violation'],
                    prefix=f"{experiment_name}_cartesian_limits_violation")

        joint_violation_mse = matrix_to_se(time, 0.0, 65.0,
                                           silo_dict['joint_limits_violation'])
        carts_violation_mse = matrix_to_se(
            time, 0.0, 65.0, silo_dict['cartesian_limits_violation'])

        fig, ax = plt.subplots()
        ax.bar(["joint limits", "cartesian limits"],
               [joint_violation_mse, carts_violation_mse])
        ax.set_title(f"{experiment_name}")
        plt.show()
