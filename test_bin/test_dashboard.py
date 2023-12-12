import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.style.use("fivethirtyeight")

x_vals, y_vals = [0], [0]


def animate(i):
    x_vals.append(x_vals[-1]+(random.random()/10))
    y_vals.append(y_vals[-1]+(random.random()/10))
    plt.cla()
    
    plt.scatter(x_vals[-1], y_vals[-1], color='red', s=100, marker='+')
    plt.plot(x_vals, y_vals)
    
    plt.ylim(-2, 2)  # Set y-axis limits
    plt.xlim(-2, 2)  # Set x-axis limits

ani = FuncAnimation(plt.gcf(), animate, interval=200)

plt.tight_layout()
plt.show()