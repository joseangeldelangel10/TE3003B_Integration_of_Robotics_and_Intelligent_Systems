import matplotlib.pyplot as plt
import pandas as pd

class Stats():

    def __init__(self,name_csv='open_loop_experiments/our_sim.csv'):
        
        self.name_csv = name_csv
        self.df = pd.read_csv(name_csv)
        if name_csv == 'open_loop_experiments/our_sim.csv':
            self.exp1lineal = self.df.loc[abs(self.df["x_f"]-0.2) <= 0.1]
            self.exp2lineal = self.df.loc[abs(self.df["x_f"]-0.4) <= 0.1]
            self.exp3lineal = self.df.loc[abs(self.df["x_f"]-0.6) <= 0.1]
            self.exp1ang = self.df.loc[abs(self.df["th_f"]-0.6) <= 0.1]
            self.exp2ang = self.df.loc[abs(self.df["th_f"]-1.2) <= 0.1]
            self.exp3ang = self.df.loc[abs(self.df["th_f"]-4.7) <= 0.1]
        elif name_csv == 'open_loop_experiments/physical.csv':
            self.exp1lineal = self.df.loc[abs(self.df["x_f"]-0.4) <= 0.1]
            self.exp2lineal = self.df.loc[abs(self.df["x_f"]-0.8) <= 0.1]
            self.exp3lineal = self.df.loc[abs(self.df["x_f"]-1.3) <= 0.1]
            self.exp1ang = self.df.loc[abs(self.df["th_f"]-1.6) <= 0.1]
            self.exp2ang = self.df.loc[abs(self.df["th_f"]-3.1) <= 0.1]
            self.exp3ang = self.df.loc[abs(self.df["th_f"]-4.7) <= 0.1]
        elif name_csv == 'open_loop_experiments/mr_sim.csv':
            self.exp1lineal = self.df.loc[abs(self.df["x_f"]-0.4) <= 0.1]
            self.exp2lineal = self.df.loc[abs(self.df["x_f"]-0.4) <= 0.1]
            self.exp3lineal = self.df.loc[abs(self.df["x_f"]-0.6) <= 0.1]
            self.exp1ang = self.df.loc[abs(self.df["th_f"]-1.6) <= 0.1]
            self.exp2ang = self.df.loc[abs(self.df["th_f"]-3.1) <= 0.1]
            self.exp3ang = self.df.loc[abs(self.df["th_f"]-4.7) <= 0.1]
        elif name_csv == 'closed_loop_experiments/our_sim.csv':
            self.exp1lineal = self.df.loc[abs(self.df["x_f"]-0.4) <= 0.1]
            self.exp2lineal = self.df.loc[abs(self.df["x_f"]-0.4) <= 0.1]
            self.exp3lineal = self.df.loc[abs(self.df["x_f"]-0.6) <= 0.1]
            self.exp1ang = self.df.loc[abs(self.df["th_f"]-1.6) <= 0.1]
            self.exp2ang = self.df.loc[abs(self.df["th_f"]-3.1) <= 0.1]
            self.exp3ang = self.df.loc[abs(self.df["th_f"]-4.7) <= 0.1]
        elif name_csv == 'closed_loop_experiments/physical.csv':
            self.exp1lineal = self.df.loc[abs(self.df["x_f"]-0.4) <= 0.1]
            self.exp2lineal = self.df.loc[abs(self.df["x_f"]-0.4) <= 0.1]
            self.exp3lineal = self.df.loc[abs(self.df["x_f"]-0.6) <= 0.1]
            self.exp1ang = self.df.loc[abs(self.df["th_f"]-1.6) <= 0.1]
            self.exp2ang = self.df.loc[abs(self.df["th_f"]-3.1) <= 0.1]
            self.exp3ang = self.df.loc[abs(self.df["th_f"]-4.7) <= 0.1]
        elif name_csv == 'closed_loop_experiments/mr_sim.csv':
            self.exp1lineal = self.df.loc[abs(self.df["x_f"]-0.4) <= 0.1]
            self.exp2lineal = self.df.loc[abs(self.df["x_f"]-0.4) <= 0.1]
            self.exp3lineal = self.df.loc[abs(self.df["x_f"]-0.6) <= 0.1]
            self.exp1ang = self.df.loc[abs(self.df["th_f"]-1.6) <= 0.1]
            self.exp2ang = self.df.loc[abs(self.df["th_f"]-3.1) <= 0.1]
            self.exp3ang = self.df.loc[abs(self.df["th_f"]-4.7) <= 0.1]


        print(self.exp3lineal)
        

        self.mux = []
        self.muy = []
        self.muth = []

        self.sigmax = []
        self.sigmay = []
        self.sigmath = []

    """
    def download_values(self):

        for i in range(len(self.data)):
            if self.data[i,5] == 1:
                self.xf[0].append(self.data[i,4])
                self.yf[0].append(self.data[i,6])
            elif self.data[i,5] == 2:
                self.xf[1].append(self.data[i,4])
                self.yf[1].append(self.data[i,6])
            elif self.data[i,5] == 3:
                self.xf[2].append(self.data[i,4])
                self.yf[2].append(self.data[i,6])
            elif abs(self.data[i,3] - 1.57079633) < 0.01:
                self.thf[0].append(self.data[i,2])
            elif abs(self.data[i,3] - 3.14159265358979) < 0.01:
                self.thf[1].append(self.data[i,2])
            elif abs(self.data[i,3] + 1.57079633) < 0.01:
                self.thf[2].append(self.data[i,2])

    def statisic_values(self):
        for i in range(3):
            
            self.mux = None
            self.muy = None
            self.muth = None

            self.sigmax = None
            self.sigmay = None
            self.sigmath = None
        

    def plot_figures(self):

        plt.figure()
        plt.scatter(self.xf[0], self.yf[0], marker="*", c="blue")
        plt.scatter(self.xf[1], self.yf[1], marker="*", c="red")
        plt.scatter(self.xf[2], self.yf[2], marker="*",c ="green")
        plt.title('Dispersion')

        plt.figure()
        plt.hist(self.xf[0])
        plt.hist(self.xf[1])
        plt.hist(self.xf[2])
        plt.title('Histograma en x')

        plt.figure()
        plt.hist(self.yf[0])
        plt.hist(self.yf[1])
        plt.hist(self.yf[2])
        plt.title('Histograma en y')

        plt.figure()
        plt.hist(self.thf[0])
        plt.hist(self.thf[1])
        plt.hist(self.thf[2])
        plt.title('Histograma en theta')
        plt.show()
    
    def main(self):
        self.download_values()
        self.plot_figures()
    """

if __name__ == '__main__':

    stat1stexperiment = Stats()
    #stat1stexperiment.main()
    