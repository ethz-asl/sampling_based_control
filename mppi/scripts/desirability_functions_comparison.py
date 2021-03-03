import random

from scipy.stats import logistic
import numpy as np
import matplotlib.pyplot as plt

figure, ax = plt.subplots()
ax.set_title("Desirability functions")
ax.set_xlabel("cost-to-go")
ax.set_ylabel("desirability")

max_cost = 100000
min_cost = 100
rollouts = 2000
gamma = 1

# cost_vector = np.random.chisquare(2, size=(rollouts, ))
cost_vector = np.random.uniform(min_cost, max_cost, size=(rollouts, ))
cost_vector.sort()

des_exp1 = np.exp(-(cost_vector - min(cost_vector)) /
                  (gamma * min(cost_vector)))
des_exp1 = des_exp1 / sum(des_exp1)
# des_exp2 = np.exp(-cost_vector/np.mean(cost_vector)); des_exp2 = des_exp2/sum(des_exp2)
des_exp2 = np.exp(-(cost_vector - min(cost_vector) / np.mean(cost_vector)))
des_exp2 = des_exp2 / sum(des_exp2)

des_log = logistic.cdf(-cost_vector, -np.mean(cost_vector),
                       np.min(cost_vector) * 100)

ax.plot(cost_vector, des_exp1, 'o-', label="des exp min")
ax.plot(cost_vector, des_exp2, 'x-', label="des exp mean")
ax.axvspan(min_cost,
           min_cost + (max_cost - min_cost) * 0.01,
           facecolor='b',
           alpha=0.2)
ax.grid()
ax.legend()
print(des_exp2[0])
# ax.plot(cost_vector, des_log, 'p')

# ax.hist(cost_vector, bins=100, density=True, alpha=0.5, stacked=True)
# ax.plot(range(-100, 100), logistic.cdf(range(-100, 100), -2, 20), 'p')
plt.show()
