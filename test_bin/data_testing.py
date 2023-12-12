import pandas as pd

data = pd.read_csv(r"C:\Users\Max\Documents\MOM\ws2324\ICT\xy_data.csv")

x = data["x_value"]
y = data["y_value"]

print(x.tail(1).values[0],y.tail(1).values[0])