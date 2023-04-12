import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

name_csv = 'our_sim.csv'

df = pd.read_csv(name_csv)

data = df.values

xf = data[:,4]
yf = data[:,6]


plt.figure()
plt.scatter(xf, yf, marker="*")
plt.title('Dispersion')

plt.figure()
plt.hist(xf)
plt.title('Distribucion en x')

plt.figure()
plt.hist(yf)
plt.title('Distribucion en y')


plt.show()
