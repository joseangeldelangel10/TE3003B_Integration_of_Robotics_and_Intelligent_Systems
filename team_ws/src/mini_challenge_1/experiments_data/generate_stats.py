import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

class Stats():

    def __init__(self,name_csv='open_loop_experiments/physical.csv'):
        
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
            self.exp3ang = self.df.loc[abs(self.df["th_f"]-4.8) <= 0.1]
        elif name_csv == 'open_loop_experiments/mr_sim.csv':
            """
            self.exp1lineal = self.df.loc[abs(self.df["x_f"]-0.4) <= 0.1]
            self.exp2lineal = self.df.loc[abs(self.df["x_f"]-0.4) <= 0.1]
            self.exp3lineal = self.df.loc[abs(self.df["x_f"]-0.6) <= 0.1]
            self.exp1ang = self.df.loc[abs(self.df["th_f"]-1.6) <= 0.1]
            self.exp2ang = self.df.loc[abs(self.df["th_f"]-3.1) <= 0.1]
            self.exp3ang = self.df.loc[abs(self.df["th_f"]-4.7) <= 0.1]
            """
            pass
        elif name_csv == 'closed_loop_experiments/our_sim.csv':
            self.exp1lineal = self.df.loc[abs(self.df["x_f"]-1) <= 0.1]
            self.exp2lineal = self.df.loc[abs(self.df["x_f"]-2) <= 0.1]
            self.exp3lineal = self.df.loc[abs(self.df["x_f"]-3) <= 0.1]
            self.exp1ang = self.df.loc[abs(self.df["th_f"]-1.57) <= 0.1]
            self.exp2ang = self.df.loc[abs(self.df["th_f"]-3.14) <= 0.1]
            self.exp3ang = self.df.loc[abs(self.df["th_f"]+1.57) <= 0.1]
        elif name_csv == 'closed_loop_experiments/physical.csv':
            self.exp1lineal = self.df.loc[abs(self.df["x_f"]-1) <= 0.1]
            self.exp2lineal = self.df.loc[abs(self.df["x_f"]-2) <= 0.2]
            self.exp3lineal = self.df.loc[abs(self.df["x_f"]-3) <= 0.3]
        elif name_csv == 'closed_loop_experiments/mr_sim.csv':
            """
            self.exp1lineal = self.df.loc[abs(self.df["x_f"]-0.4) <= 0.1]
            self.exp2lineal = self.df.loc[abs(self.df["x_f"]-0.4) <= 0.1]
            self.exp3lineal = self.df.loc[abs(self.df["x_f"]-0.6) <= 0.1]
            self.exp1ang = self.df.loc[abs(self.df["th_f"]-1.6) <= 0.1]
            self.exp2ang = self.df.loc[abs(self.df["th_f"]-3.1) <= 0.1]
            self.exp3ang = self.df.loc[abs(self.df["th_f"]-4.7) <= 0.1]
            """
            pass


        self.xf = [[np.array(self.exp1lineal["x_f"])],[np.array(self.exp2lineal["x_f"])],[np.array(self.exp3lineal["x_f"])]]
        self.yf = [[np.array(self.exp1lineal["y_f"])],[np.array(self.exp2lineal["y_f"])],[np.array(self.exp3lineal["y_f"])]]
        self.thf = [[np.array(self.exp1ang["th_f"])],[np.array(self.exp2ang["th_f"])],[np.array(self.exp3ang["th_f"])]]

        self.mux = []
        self.muy = []
        self.muth = []

        self.sigmax = []
        self.sigmay = []
        self.sigmath = []

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
        self.plot_figures()

if __name__ == '__main__':

    stat1stexperiment = Stats()
    stat1stexperiment.main()
    