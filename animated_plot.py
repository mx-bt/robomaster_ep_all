import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.style.use("fivethirtyeight")

def animate(i):
    try:
        data = pd.read_csv(r"C:\Users\Max\Documents\MOM\ws2324\ICT\xy_data.csv")

        y = data["x_value"]
        x = data["y_value"]

        plt.cla()

        plt.scatter((x.tail(1).values[0]),(y.tail(1).values[0]), label='Señor Javier', color='red', s=300, marker='+',zorder=1)
        plt.plot(x, y,zorder=2)

        plt.ylim(-5, 5)  # Set y-axis limits
        plt.xlim(-15, 5)  # Set x-axis limits

        plt.legend(loc='upper left')
        plt.tight_layout()
    except KeyboardInterrupt:
        plt.close()
        raise

ani = FuncAnimation(plt.gcf(), animate, interval=200)
plt.tight_layout()
plt.show()