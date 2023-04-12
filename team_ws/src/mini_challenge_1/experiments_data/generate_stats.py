import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

name_csv = 'our_sim.csv'

df = pd.read_csv(name_csv)

data = df.values

xf = data[:,4]
yf = data[:,6]


fig, ax = plt.subplots()
ax.scatter(xf, yf, marker="*")
ax.hist
plt.show()
