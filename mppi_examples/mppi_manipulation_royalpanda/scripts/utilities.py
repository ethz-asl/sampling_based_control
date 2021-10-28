import os
import re
import sys
import functools
import pandas as pd
import numpy as np
import signal_logger

import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns

from tkinter import Tk
from tkinter import filedialog as fd

import ipywidgets as widgets
from IPython.display import display

#############################
# Global plotting settings
#############################
stylesheet = os.path.join(os.path.dirname(__file__), "paper.mplstyle")

###############################
# Utilities
###############################


def reload_plt_style():
    plt.style.use(stylesheet)


class ExperimentType:
    NO_FILTER = "no_filter"
    FILTER_OUT = "filter_out"
    FILTER_IN = "filter_in"
    FILTER_IN_OUT = "filter_in_out"
    PASSIVITY_ZBF = "zbf"

    @classmethod
    def get_labels(cls):
        return ["NO_FILTER", "FILTER_OUT", "FILTER_IN", "FILTER_IN_OUT"]


def experiment_type_from_name(name):
    if "filter_in_out" in name:
        return ExperimentType.FILTER_IN_OUT
    elif "filter_in" in name:
        return ExperimentType.FILTER_IN
    elif "filter_out" in name:
        return ExperimentType.FILTER_OUT
    elif "zbf" in name:
        return ExperimentType.PASSIVITY_ZBF
    else:
        return ExperimentType.NO_FILTER


def get_rollouts_from_name(name):
    return re.sub("[^0-9]", "", name)


def get_object_from_name(name):
    object_types = [
        'shelf', 'door', 'microwave', 'drawer', 'valve', 'new_shelf'
    ]
    for object_type in object_types:
        if object_type in name:
            return object_type
    return 'none'


def get_data(root_dir, required_fields, log_prefix=""):
    LOG_FILES = get_files(root_dir, log_prefix)
    LOG_FILES_PRINT = '\n'.join(LOG_FILES)
    print(f"""
    Root dir: 
    {root_dir}
    Prefix: 
    {log_prefix}
    Log files: 
    {LOG_FILES_PRINT}
    """)

    data = {'experiment_name': []}
    for experiment_idx, file in enumerate(LOG_FILES):
        print(f"{experiment_idx} processed out of {len(LOG_FILES)}")
        experiment_name = file.split('/')[-1].replace('.silo', '')
        silo = signal_logger.Silo(file)

        silo_dict = to_dictionary(silo, required_fields)

        data['experiment_name'].append(experiment_name)
        for key, value in silo_dict.items():
            if experiment_idx == 0:
                data[key] = [value]
            else:
                data[key].append(value)
    return data


def process_data(data, final_time):
    """
    final_time: up to which time the metric should be computed for each experiment
    """
    data['joint_limits_violation_se'] = []
    data['cartesian_limits_violation_se'] = []
    data['average_wrench_norm'] = []
    data['experiment_type'] = []
    data['power_cost'] = []
    data['object_type'] = []
    data['x'] = []
    data['average_stage_cost'] = []
    data['wrench_norm'] = []
    data['rollouts'] = []
    data['dissipated_power'] = []

    first_field = list(data.keys())[0]
    num_experiments = len(data[first_field])
    print(f"Processing {num_experiments} experiments.")
    for experiment_idx in range(num_experiments):
        experiment_name = data['experiment_name'][experiment_idx]
        print(f"Processing {experiment_name}")

        time = data['sim_time'][experiment_idx]
        print(f"t start = {time[0]}s, t end = {time[-1]}s")

        joint_violation_se = matrix_to_se(
            time, 0.0, final_time,
            data['joint_limits_violation'][experiment_idx])
        carts_violation_se = matrix_to_se(
            time, 0.0, final_time,
            data['cartesian_limits_violation'][experiment_idx])
        average_wrench = average_col_norm(
            time,
            0.0,
            final_time,
            data['external_wrench_filtered'][experiment_idx],
            threshold=0.0)
        wrench_norm = col_norm(
            time, 0.0, final_time,
            data['external_wrench_filtered'][experiment_idx])
        average_stage_cost = np.mean(data['stage_cost'][experiment_idx])

        data['joint_limits_violation_se'].append(joint_violation_se)
        data['cartesian_limits_violation_se'].append(carts_violation_se)
        data['average_wrench_norm'].append(average_wrench)
        data['experiment_type'].append(
            experiment_type_from_name(experiment_name))
        data['power_cost'].append("power" in experiment_name)
        data['object_type'].append(get_object_from_name(experiment_name))
        data['x'].append('data')
        data['average_stage_cost'].append(average_stage_cost)
        data['wrench_norm'].append(wrench_norm)
        data['rollouts'].append(get_rollouts_from_name(experiment_name))
        data['dissipated_power'].append(
            np.sum(data['power_from_interaction'][experiment_idx]))
    return data


###################
#  Plotting
###################
def set_facetgrid_style(g, legend=False, title=True):
    for i, (col_val, ax) in enumerate(g.axes_dict.items()):
        ax.set_title(col_val if title else "", fontsize=60)
        ax.yaxis.label.set_size(50)
        ax.tick_params(axis='y', which='major', labelsize=40)
        ax.tick_params(axis='x',
                       which='both',
                       bottom=False,
                       top=False,
                       labelbottom=False)
        ax.grid(True)
        if i == len(g.axes_dict.items()) - 1 and legend:
            ax.legend(fontsize=48)
        g.fig.set_size_inches(42.5, 12.5)


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


def add_subplot_axes(ax, rect, facecolor='w'):
    fig = ax.figure
    box = ax.get_position()
    width = box.width
    height = box.height
    inax_position = ax.transAxes.transform(rect[0:2])
    transFigure = fig.transFigure.inverted()
    infig_position = transFigure.transform(inax_position)
    x = infig_position[0]
    y = infig_position[1]
    width *= rect[2]
    height *= rect[3]  # <= Typo was here
    subax = fig.add_axes([x, y, width, height],
                         facecolor=facecolor)  # matplotlib 2.0+
    x_labelsize = subax.get_xticklabels()[0].get_size()
    y_labelsize = subax.get_yticklabels()[0].get_size()
    x_labelsize *= rect[2]**0.5
    y_labelsize *= rect[3]**0.5
    subax.xaxis.set_tick_params(labelsize=x_labelsize)
    subax.yaxis.set_tick_params(labelsize=y_labelsize)
    return subax


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
    return np.square(m[i1:i2]).sum()


def average_col_norm(t, t_start, t_end, m, threshold=0):
    norms = col_norm(t, t_start, t_end, m)
    return np.mean(norms[norms > threshold])


def col_norm(t, t_start, t_end, m):
    # Assume each row in the matrix as a 1-1 correspondence to time stamps
    # Use time start and time end to crop them matrix and compute MSE only on this portion
    i1 = sum(t < t_start)
    i2 = sum(t < t_end)

    if i2 > m.shape[0]:
        raise NameError(
            "The index of the final time is larger than rows in the matrix")

    return np.sqrt(np.diagonal(np.dot(m[i1:i2, :], m[i1:i2, :].T)))


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


def replace_legend_labels(ax, old_labels, new_labels, fontsize=10):
    """ 
    Replace old labels (if in the axis) with the new ones
    """

    if (len(old_labels) != len(new_labels)):
        raise NameError("old labels and new labels have different size.")

    # get legend and loop through the labels
    l = ax.legend(fontsize=fontsize)
    for text_idx, text in enumerate(l.get_texts()):
        text = text.get_text()
        new_text = text
        if text in old_labels:
            new_text = new_labels[old_labels.index(text)]
        l.get_texts()[text_idx].set_text(new_text)
