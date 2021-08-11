import os
import numpy as np
import signal_logger
import matplotlib.pyplot as plt

plt.rcParams['axes.grid'] = True

###############################
# Definitions
###############################
import rospkg
rospack = rospkg.RosPack()
ROOT_DIR = rospack.get_path("mppi_manipulation")
LOG_FILE = os.path.join(ROOT_DIR, "data", "logs",
                        "mppi_manipulation_with_filter.silo")
REQUIRED_FIELDS = [
    "log/sim_time", "log/input", "log/input_filtered",
    "log/joint_limits_violation", "log/solver/rollouts/min_cost",
    "log/solver/rollouts/max_cost"
]


###############################
# Utilities
###############################
def print_keys(logger: signal_logger.Silo):
    print(f"Logger contains the following keys ({len(silo.keys())}):")
    for key in silo.keys():
        print(f"[{key}]")


def eval_field(logger: signal_logger.Silo, prefix: str):
    """
    Evaluate a field specified by its prefix in the silo logger
    Vector are transformed to scalar field appended by _<idx>. If the pattern is found
    then a matrix containing each index time series as column is returned
    :param logger: silo logger
    :param prefix: prefix or full_name
    :return:
    """
    keys = logger.keys()
    entries = []
    found = False
    for key in keys:
        if key.startswith(prefix):
            postfix = key.replace(prefix, '')
            # Scalar case
            if postfix == '':
                return logger.get_signal(prefix).values
            # Good candidate to be a vector but other entries with similar prefixes might be there
            else:
                postfixes = postfix.split("_")
                # The prefix is not followed by nothing else the vector index postfix
                if len(postfixes) == 2:
                    found = True
                    entries.append(eval(postfixes[-1]))

    if not found:
        print(f"Failed to find entry with prefix [{prefix}]")
        return None

    cols = max(entries) + 1
    rows = logger.get_signal(prefix + "_0").values.size
    matrix = np.zeros((rows, cols))
    for col in range(cols):
        matrix[:, col] = logger.get_signal(prefix + f"_{col}").values
    return matrix


def to_dictionary(logger: signal_logger.Silo, fields):
    """ Given a list of prefixes (see eval_filed) returns a representation of the logs as a dictionary """
    d = {}
    for field in fields:
        dict_field = field.split('/')[-1]
        d[dict_field] = eval_field(logger, prefix=field)
    return d


def matrix_plot(m: np.ndarray, prefix="value"):
    """
    Plot each row of a matrix in the same plot
    :param m:
    :param prefix:
    :return:
    """
    fig, ax = plt.subplots()
    for i in range(m.shape[1]):
        ax.plot(m[:, i], label=f"{prefix}_{i}")
    #ax.grid()
    ax.legend()


###############################
# Main
###############################

if __name__ == "__main__":
    silo = signal_logger.Silo(LOG_FILE)
    print_keys(silo)
    silo_dict = to_dictionary(silo, REQUIRED_FIELDS)

    fig1, ax1 = plt.subplots()
    ax1.plot(silo_dict['sim_time'], silo_dict['input'][:, 0], label="input")
    ax1.plot(silo_dict['sim_time'],
             silo_dict['input_filtered'][:, 0],
             label="input_filtered")
    #ax1.grid()
    ax1.legend()

    # ax.plot(silo_dict['sim_time'],
    #         silo_dict['joint_limits_violation'][:, 0],
    #         label="constraint_violation")

    fig2, ax2 = plt.subplots()
    ax2.plot(silo_dict['sim_time'], silo_dict['min_cost'], label="min_cost")
    #ax2.grid()
    ax2.legend()

    matrix_plot(silo_dict['input'], prefix="input")
    matrix_plot(silo_dict['input_filtered'], prefix="input_filtered")
    matrix_plot(silo_dict['joint_limits_violation'],
                prefix="joint_limits_violation")
    #plt.grid()
    plt.legend()
    plt.show()
