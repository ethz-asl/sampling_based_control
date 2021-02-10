#! /usr/bin/env python

import os
import rospy
from rospkg import RosPack
import uuid
import pandas as pd
from mppi_ros.msg import Data


class DataRecorder:
    def __init__(self, experiment_id = "exp"):
        self.data_dict = {"experiment_id": experiment_id + str(uuid.uuid4()),
                          "horizon": 0.0,
                          "nr_rollouts": 0,
                          "filter_window": 0,
                          "tree_search": False,
                          "stage_cost": [[]],
                          "weights": [[]],
                          "step_count": [[]]}
        self.first_call = True
        self.data_subscriber = rospy.Subscriber("/mppi_data", Data, self.data_callback, queue_size=10)
        self.step_count = 0
        self.initial_step_count = 0

        self.csv_file = os.path.join(RosPack().get_path("mppi_ros"), "log", "record.csv")
        rospy.loginfo("Writing to {}".format(self.csv_file))

    def data_callback(self, data):
        if len(data.weights.array) == 0:
            return

        if self.first_call:
            rospy.loginfo("Received first message: horizon is: {}".format(data.config.horizon))
            self.data_dict["horizon"] = data.config.horizon
            self.data_dict["nr_rollouts"] = data.config.nr_rollouts
            self.data_dict["filter_window"] = data.config.filter_window
            self.data_dict["tree_search"] = data.config.tree_search
            self.initial_step_count = data.step_count
            self.first_call = False

        self.data_dict["stage_cost"][0].append(data.stage_cost)
        self.data_dict["weights"][0].append(data.weights.array)
        self.data_dict["step_count"][0].append(data.step_count - self.initial_step_count)

    def save(self):
        df = pd.DataFrame(self.data_dict)
        df.to_csv(self.csv_file, mode='a', header=False)


if __name__ == "__main__":
    rospy.init_node("record_data")
    experiment_id = rospy.get_param("~experiment_id")
    recorder = DataRecorder(experiment_id)
    rospy.spin()
    recorder.save()
