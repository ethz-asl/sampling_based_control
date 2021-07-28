import matplotlib.pyplot as plt
import signal_logger

silo = signal_logger.Silo(
    "/home/giuseppe/.ros/silo_20210728_17-54-32_00017.silo")
print("Silo containing the following keys: {}".format(silo.keys()))

joint_violation_0 = silo.get_signal('/log/joint_limits_violation_0').values
input_0 = silo.get_signal('/log/input_0').values
input_filt_0 = silo.get_signal('/log/input_filt_0').values
sim_time = silo.get_signal('/log/sim_time').values

fig, ax = plt.subplots()
ax.plot(sim_time, input_0, label="input")
ax.plot(sim_time, input_filt_0, label="input_filt")
ax.plot(sim_time, joint_violation_0, label="constraint_violation")
plt.legend()
plt.grid()
plt.show()

#
# def acc_norm(t):
#     from pylab import norm  # don't do that in general, this is just for the example
#     return norm([acc_x.value(t), acc_y.value(t), acc_z.value(t)])
#
# times = acc_x.times
# acc_norm_signal = signal_logger.Signal(times, [acc_norm(t) for t in times], 'acc_norm')
# acc_norm_signal.plot()