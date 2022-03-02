# https://www.codesansar.com/python-programming-examples/plot-circle-using-numpy-matplotlib.htm
import numpy as np
from matplotlib import pyplot as plt

# Creating equally spaced 100 data in range 0 to 2*pi
theta = np.linspace(0, 2 * np.pi, 100)

# Setting radius
radius = 5

center = [5,6]

# Generating x and y data
x = radius * np.cos(theta) + center[0]
y = radius * np.sin(theta) - center[1]

# Plotting
plt.plot(x, y,color="peru")
plt.axis('equal')
plt.title('Circle')
plt.show()
