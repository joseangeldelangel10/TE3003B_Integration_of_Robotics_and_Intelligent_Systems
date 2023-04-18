import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
from scipy.stats import norm
import statistics as stats

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


        self.xf1 = np.array(self.exp1lineal["x_f"])
        self.xf2 = np.array(self.exp2lineal["x_f"])
        self.xf3 = np.array(self.exp3lineal["x_f"])
        self.yf1 = np.array(self.exp1lineal["y_f"])
        self.yf2 = np.array(self.exp2lineal["y_f"])
        self.yf3 = np.array(self.exp3lineal["y_f"])
        self.thf1 = np.array(self.exp1ang["th_f"])
        self.thf2 = np.array(self.exp2ang["th_f"])
        self.thf3 = np.array(self.exp3ang["th_f"])

        self.mux = []
        self.muy = []
        self.muth = []

        self.sigmax = []
        self.sigmay = []
        self.sigmath = []

        self.cov = None

    def statisic_values(self):
        for i in range(3):
            
            self.mux = None
            self.muy = None
            self.muth = None

            self.sigmax = None
            self.sigmay = None
            self.sigmath = None

    def confidence_ellipse(self,x, y, ax, n_std=2.0, facecolor='none', **kwargs):
        """
        Create a plot of the covariance confidence ellipse of 'x' and 'y'
        Parameters
        ----------
        x, y : array_like, shape (n, )
            Input data.
        ax : matplotlib.axes.Axes
            The axes object to draw the ellipse into.
        n_std : float
            The number of standard deviations to determine the ellipse's radiuses.
        Returns
        -------
        matplotlib.patches.Ellipse
        Other parameters
        ----------------
        kwargs : '~matplotlib.patches.Patch' properties
        """
        if x.size != y.size:
            raise ValueError("x and y must be the same size")

        cov = np.cov(x, y)
        pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
        # Using a special case to obtain the eigenvalues of this
        # two-dimensionl dataset.
        ell_radius_x = np.sqrt(1 + pearson)
        ell_radius_y = np.sqrt(1 - pearson)
        ellipse = Ellipse((0, 0),
            width=ell_radius_x * 2,
            height=ell_radius_y * 2,
            facecolor=facecolor,
            **kwargs)

        # Calculating the stdandard deviation of x from
        # the squareroot of the variance and multiplying
        # with the given number of standard deviations.
        scale_x = np.sqrt(cov[0, 0]) * n_std
        mean_x = np.mean(x)

        # calculating the stdandard deviation of y ...
        scale_y = np.sqrt(cov[1, 1]) * n_std
        mean_y = np.mean(y)

        transf = transforms.Affine2D() \
            .rotate_deg(45) \
            .scale(scale_x, scale_y) \
            .translate(mean_x, mean_y)

        ellipse.set_transform(transf + ax.transData)
        return ax.add_patch(ellipse)
    
    def plot_figures(self):

        fig, ax = plt.subplots()

        ax.scatter(self.xf1,self.yf1,marker="*", c="blue")
        self.confidence_ellipse(np.array(self.xf1), np.array(self.yf1), ax, edgecolor='blue')

        ax.scatter(self.xf2,self.yf2,marker="*", c="red")
        self.confidence_ellipse(np.array(self.xf2), np.array(self.yf2), ax, edgecolor='red')

        ax.scatter(self.xf3,self.yf3,marker="*", c="green")
        self.confidence_ellipse(np.array(self.xf3), np.array(self.yf3), ax, edgecolor='green')
            
        plt.figure()
        plt.hist(self.xf1)
        plt.hist(self.xf2)
        plt.hist(self.xf3)
        plt.title('Histograma en los 3 experimentos lineales en x')


        plt.figure()
        plt.hist(self.yf1)
        plt.hist(self.yf2)
        plt.hist(self.yf3)
        plt.title('Histograma en los 3 experimentos lineales en y')

        plt.figure()
        plt.hist(self.thf1)
        plt.hist(self.thf2)
        plt.hist(self.thf3)
        plt.title('Histograma en los 3 experimentos lineales en theta')

        plt.show()

    
    def main(self):
        self.plot_figures()

if __name__ == '__main__':

    stat1stexperiment = Stats()
    stat1stexperiment.main()
    