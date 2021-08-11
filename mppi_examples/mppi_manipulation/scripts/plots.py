import numpy as np
import matplotlib.pyplot as plt

x = np.linspace(0, 100, 100)
y = 1 - np.exp(-x / 10)
plt.plot(x, y)
plt.grid()

plt.show()
