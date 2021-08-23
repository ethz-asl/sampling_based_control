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
ROOT_DIR = rospack.get_path("mppi_manipulation_royalpanda")
LOG_FILE = os.path.join(ROOT_DIR, "logs", "filter.silo")
REQUIRED_FIELDS = [
    "log/sim_time", "log/stage_cost", "log/torque_command",
    "log/cartesian_limits_violation", "log/joint_limits_violation",
    "log/solver/rollouts/min_cost"
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


def matrix_plot(t, m: np.ndarray, prefix="value"):
    fig, ax = plt.subplots()
    for i in range(m.shape[1]):
        ax.plot(t, m[:, i], label=f"{prefix}_{i}")
    ax.legend()


def matrix_to_se(t, t_start, t_end, m):
    # Assume each row in the matrix as a 1-1 correspondence to time stamps
    # Use time start and time end to crop them matrix and compute MSE only on this portion
    i1 = sum(t < t_start)
    i2 = sum(t < t_end)

    if i2 > m.shape[0]:
        raise NameError(
            "The index of the final time is larger than rows in the matrix")
    return np.square(m[i1:i2, :]).sum()


def scalar_plot(t, n, prefix="value"):
    fig, ax = plt.subplots()
    ax.plot(t, n, label=prefix)
    ax.legend()


###############################
# Main
###############################

if __name__ == "__main__":
    silo = signal_logger.Silo(LOG_FILE)
    print_keys(silo)
    silo_dict = to_dictionary(silo, REQUIRED_FIELDS)
    time = silo_dict['sim_time']
    print(min(time))
    print(max(time))

    scalar_plot(time, silo_dict['stage_cost'], prefix="stage_cost")
    scalar_plot(time, silo_dict['min_cost'], prefix="min_cost")
    matrix_plot(time, silo_dict['torque_command'], prefix="torque_command")
    matrix_plot(time,
                silo_dict['joint_limits_violation'],
                prefix="joint_limits_violation")
    matrix_plot(time,
                silo_dict['cartesian_limits_violation'],
                prefix="cartesian_limits_violation")

    joint_violation_mse = matrix_to_se(time, 0.0, 65.0,
                                       silo_dict['joint_limits_violation'])
    carts_violation_mse = matrix_to_se(time, 0.0, 65.0,
                                       silo_dict['cartesian_limits_violation'])

    fig, ax = plt.subplots()
    ax.bar(["joint limits", "cartesian limits"],
           [joint_violation_mse, carts_violation_mse])
    plt.show()
