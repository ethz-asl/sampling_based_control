import os
import numpy as np
import signal_logger
import matplotlib.pyplot as plt

plt.rcParams['axes.grid'] = True


###############################
# Utilities
###############################
def get_files(path, prefix=""):
    f = []
    for (dirpath, dirnames, filenames) in os.walk(path):
        for file in filenames:
            if file.startswith(prefix):
                f.append(os.path.join(dirpath, file))
    return f


def print_keys(silo: signal_logger.Silo):
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