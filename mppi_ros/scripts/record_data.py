#! /usr/bin/env python

import os
import rospy
from rospkg import RosPack
import uuid
import pandas as pd
import numpy as np
from mppi_ros.msg import Data
from scipy.ndimage.filters import gaussian_filter1d

# To make sure timestamps are uniform through experiments
from scipy.interpolate import interp1d


class DataRecorder:
    def __init__(self, experiment_id="exp"):
        self.data_dict = {
            "id": [],
            "index": [],
            "horizon": [],
            "nr_rollouts": [],
            "alpha": [],
            "beta": [],
            "filter_window": [],
            "tree_search": [],
            "stage_cost": [],
            "effective_samples": [],
            "time": [],
            "opt_time": []
        }
        self.first_call = True
        self.data_subscriber = rospy.Subscriber("/mppi_data",
                                                Data,
                                                self.data_callback,
                                                queue_size=10)
        self.initial_step_count = 0
        self.initial_time = 0
        self.idx = 0
        self.experiment_id = experiment_id + str(uuid.uuid4())

        self.csv_file = os.path.join(RosPack().get_path("mppi_ros"), "log",
                                     "record.csv")
        rospy.loginfo("Writing to {}".format(self.csv_file))

    def data_callback(self, data: Data):
        if len(data.weights.array) == 0:
            return

        current_time = rospy.get_rostime().to_sec()
        if self.first_call:
            self.initial_step_count = data.step_count
            self.initial_time = current_time
            self.first_call = False

        self.data_dict["id"].append(self.experiment_id)
        self.data_dict["index"].append(self.idx)
        self.data_dict["horizon"].append(data.config.horizon)
        self.data_dict["nr_rollouts"].append(data.config.nr_rollouts)
        self.data_dict["alpha"].append(data.config.alpha)
        self.data_dict["beta"].append(data.config.beta)
        self.data_dict["filter_window"].append(data.config.filter_window)
        self.data_dict["tree_search"].append(data.config.tree_search)
        self.data_dict["stage_cost"].append(data.stage_cost)
        self.data_dict["time"].append(current_time - self.initial_time)
        self.data_dict["opt_time"].append(data.time.to_sec())

        self.idx += 1

        effective_samples = 1.0 / (len(data.weights.array) * np.square(
            np.asarray(data.weights.array)).sum())
        self.data_dict["effective_samples"].append(effective_samples)

    @staticmethod
    def resize_array(l, new_size):
        assert isinstance(l, list)
        if len(l) > new_size:
            l = l[:new_size]
        elif len(l) < new_size:
            size = len(l)
            l.extend([l[-1]] * (new_size - size))

    def postprocess(self):
        # Filtering of effective samples
        self.data_dict['effective_samples'] = gaussian_filter1d(
            self.data_dict['effective_samples'], sigma=2)

        # First save controller rate
        rate = np.array(self.data_dict['opt_time'])
        rate = 1. / (rate[1:] - rate[:-1])
        rate = np.insert(rate, 0, 0)
        self.data_dict["rate"] = rate

        # Resampling of cost and timestamps: data needs to be aligned for aggregation
        t_min = min(self.data_dict['time'])
        t_max = max(self.data_dict['time'])
        t_old = self.data_dict['time']
        t_new = np.round(np.linspace(t_min, t_max,
                                     len(self.data_dict["index"])),
                         decimals=3)
        self.data_dict['time'] = t_new

        cost_int_fun = interp1d(t_old,
                                self.data_dict['stage_cost'],
                                fill_value='extrapolate')
        self.data_dict['stage_cost'] = cost_int_fun(t_new)

        samples_int_fun = interp1d(t_old,
                                   self.data_dict['effective_samples'],
                                   fill_value='extrapolate')
        self.data_dict['effective_samples'] = samples_int_fun(t_new)

    def save(self):
        df = pd.DataFrame(self.data_dict)
        if os.path.isfile(self.csv_file):
            df.to_csv(self.csv_file, mode='a', header=False)
        else:
            df.to_csv(self.csv_file)


if __name__ == "__main__":
    rospy.init_node("record_data")
    experiment_id = rospy.get_param("~experiment_id")
    recorder = DataRecorder(experiment_id)
    rospy.spin()

    recorder.postprocess()
    recorder.save()
