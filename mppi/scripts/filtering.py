import numpy as np
import matplotlib.pyplot as plt

delta = 0.02
x_left = np.linspace(0, delta, 1000)
x_right = np.linspace(delta, 1, 1000)


def fun(x, delta=0):
    return np.exp(-delta) * (delta**2 / x + 1 - delta)


y_left = fun(x_left, delta=delta)
y_right = np.exp(-x_right)

fig, ax = plt.subplots()
ax.plot(x_left, y_left, 'b')
ax.plot(x_right, y_right, 'g')
plt.show()
