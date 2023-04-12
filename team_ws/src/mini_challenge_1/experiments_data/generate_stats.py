import matplotlib.pyplot as plt
import pandas as pd

name_csv = 'our_sim.csv'

df = pd.read_csv(name_csv)

data = df.values

xf = [[],[],[]]
yf = [[],[],[]]
thf = [[],[],[]]

for i in range(len(data)):
    if data[i,5] == 1:
        xf[0].append(data[i,4])
        yf[0].append(data[i,6])
    elif data[i,5] == 2:
        xf[1].append(data[i,4])
        yf[1].append(data[i,6])
    elif data[i,5] == 3:
        xf[2].append(data[i,4])
        yf[2].append(data[i,6])
    elif abs(data[i,3] - 1.57079633) < 0.01:
        thf[0].append(data[i,2])
    elif abs(data[i,3] - 3.14159265358979) < 0.01:
        thf[1].append(data[i,2])
    elif abs(data[i,3] + 1.57079633) < 0.01:
        thf[2].append(data[i,2])


plt.figure()
plt.scatter(xf[0], yf[0], marker="*", c="blue")
plt.scatter(xf[1], yf[1], marker="*", c="red")
plt.scatter(xf[2], yf[2], marker="*",c ="green")
plt.title('Dispersion')

plt.figure()
plt.hist(xf[0])
plt.hist(xf[1])
plt.hist(xf[2])
plt.title('Histograma en x')

plt.figure()
plt.hist(yf[0])
plt.hist(yf[1])
plt.hist(yf[2])
plt.title('Histograma en y')

plt.figure()
plt.hist(thf[0])
plt.hist(thf[1])
plt.hist(thf[2])
plt.title('Histograma en theta')


plt.show()
