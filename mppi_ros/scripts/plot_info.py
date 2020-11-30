#! /usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt

from mppi_ros.msg import RolloutsInfo

plt.ion()


class RolloutsInfoPlotter():
    def __init__(self):
        self.on_launch()

    def on_launch(self):
        rospy.init_node("plotter")
        self.rollouts_info_subscriber = rospy.Subscriber("/rollouts_info", RolloutsInfo, self.draw)

        # Set up plot
        self.figure, axes = plt.subplots(2, 1)
        self.rollouts_cost_ax = axes[0]
        self.weights_ax = axes[1]
        self.rollouts_data, = self.rollouts_cost_ax.plot([], [], 'o')
        self.weights_data, = self.weights_ax.plot([], [], 'o')

        # self.ax.set_autoscaley_on(True)
        self.weights_ax.set_title = "Weights"
        self.weights_ax.set_xlabel = "rollouts"
        self.weights_ax.set_ylabel = "weights"
        self.weights_ax.set_autoscalex_on(True)
        self.weights_ax.set_ylim(0, 1)
        self.weights_ax.set_autoscaley_on(True)
        self.weights_ax.grid()

        self.rollouts_cost_ax.set_title = "Cost to go"
        self.rollouts_cost_ax.set_xlabel = "rollouts"
        self.rollouts_cost_ax.set_ylabel = "cost"
        self.rollouts_cost_ax.grid()

    def update_plot(self):
        self.rollouts_cost_ax.relim()
        self.rollouts_cost_ax.autoscale_view()

        self.weights_ax.relim()
        self.weights_ax.autoscale_view()

        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

    # Example
    def draw(self, msg):
        xdata = range(len(msg.weights))
        self.rollouts_data.set_xdata(xdata)
        self.rollouts_data.set_ydata(np.sort(msg.cost_to_go))
        self.weights_data.set_xdata(xdata)
        self.weights_data.set_ydata(np.sort(msg.weights))


plotter = RolloutsInfoPlotter()
plotter.update_plot()
while not rospy.is_shutdown():
    plotter.update_plot()
