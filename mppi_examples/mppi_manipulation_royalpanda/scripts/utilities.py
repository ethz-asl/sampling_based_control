import os
import numpy as np
import functools
import signal_logger

import ipywidgets as widgets

import matplotlib.pyplot as plt

from tkinter import Tk
from tkinter import filedialog as fd

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


def matrix_plot(t, m: np.ndarray, prefix="value", linestyle="-", axis=None):
    if not axis:
        fig, ax = plt.subplots()
    else:
        ax = axis

    for i in range(m.shape[1]):
        ax.plot(t, m[:, i], linestyle=linestyle, label=f"{prefix}_{i}")
    ax.legend()

    if not axis:
        display_save_button(fig)
    return ax


def matrix_to_se(t, t_start, t_end, m):
    # Assume each row in the matrix as a 1-1 correspondence to time stamps
    # Use time start and time end to crop them matrix and compute MSE only on this portion
    i1 = sum(t < t_start)
    i2 = sum(t < t_end)

    if i2 > m.shape[0]:
        raise NameError(
            "The index of the final time is larger than rows in the matrix")
    return np.square(m[i1:i2, :]).sum()


def average_col_norm(t, t_start, t_end, m):
    # Assume each row in the matrix as a 1-1 correspondence to time stamps
    # Use time start and time end to crop them matrix and compute MSE only on this portion
    i1 = sum(t < t_start)
    i2 = sum(t < t_end)

    if i2 > m.shape[0]:
        raise NameError(
            "The index of the final time is larger than rows in the matrix")

    return np.mean(np.sqrt(np.diagonal(np.dot(m[i1:i2, :], m[i1:i2, :].T))))


def bar_plot(index, labels, *args):
    """
    labels: how many dataset to compare
    args: list of list of values for each index. 
    Example:
      labels = ["dataset1", "dataset2"]
      index = ["A", "B"]
      args[0] = [[list of values for A db 1], [list of values for B db 2]]
      args[1] = [[list of values for A db 2], [list of values for B db 2]]
    """

    bar_width = 0.25
    numeric_index = np.arange(len(index))

    if len(args) != len(labels):
        raise NameError("Should provide as many labels as value lists")

    # Plot bar plot for each label
    fig, ax = plt.subplots()
    for label_id, label in enumerate(labels):
        # compute mean value (height) and std (error bar) for each value in index
        mean = [
            np.mean(args[label_id][index_id]) for index_id in numeric_index
        ]
        std = [np.std(args[label_id][index_id]) for index_id in numeric_index]

        # plot mean as bar plot
        ax.bar(
            numeric_index + label_id * bar_width,
            mean,
            bar_width,
            label=label,
        )

        # add error bar (marker and line set to black)
        ax.errorbar(numeric_index + label_id * bar_width,
                    mean,
                    yerr=std,
                    fmt='o',
                    ecolor='k',
                    mfc='k',
                    mec='k')

    # Make sure the index is in the center of each comparison block (bars related to the same index value)
    ax.set_xticks(numeric_index + (len(labels) - 1) * bar_width / 2)
    ax.set_xticklabels(index)
    ax.legend()

    display_save_button(fig)

    return ax


def scalar_plot(t, n, prefix="value", linestyle="-", axis=None):
    if not axis:
        fig, ax = plt.subplots()
    else:
        ax = axis
    ax.plot(t, n, linestyle=linestyle, label=prefix)
    ax.legend()

    if not axis:
        display_save_button(fig)
    return ax


# Ack since in chrome the builtin download button does not work
def display_save_button(figure):
    button = widgets.Button(description="Save Figure")
    display(button)
    button.on_click(functools.partial(save_figure, figure=figure))


def save_figure(b, figure):
    Tk().withdraw()  # create and withdraw root window
    save_name = fd.asksaveasfilename()
    if save_name:
        figure.savefig(save_name)
