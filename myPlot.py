import numpy as np
import matplotlib.pyplot as plt

class myPlot:
    def __init__(self):
        pass
    
    @classmethod
    def plot3(cls,fig_num,x,y1,y2,title,xlabel,ylabel,legend):
          
        plt.figure(fig_num)
        plt.subplot(311)
        plt.plot(x,y1[0,:])
        plt.plot(x,y2[0,:],'--')
        plt.xlabel(xlabel)
        plt.ylabel(ylabel[0])
        plt.grid(True)
        

        plt.subplot(312)
        plt.plot(x,y1[1,:])
        plt.plot(x,y2[1,:],'--')
        plt.xlabel(xlabel)
        plt.ylabel(ylabel[1])                
        plt.grid(True)

        plt.subplot(313)
        plt.plot(x,y1[2,:])
        plt.plot(x,y2[2,:],'--')
        plt.xlabel(xlabel)
        plt.ylabel(ylabel[2])
        plt.legend(legend)
        plt.grid(True)
        plt.suptitle(title)
