from pylab import *
import matplotlib.pyplot as plt
import numpy as np


class Plotter:
    td = []
    lines = []
    columnIDs = []
    figureId = 0
    maxPoints = 1000;
    
    def __init__(self, td, figureId, maxPoints):
        self.td = td
        self.figureId = figureId
        figure(self.figureId)
        self.maxPoints = maxPoints
        grid(b=1)

    def initAsLivePlot(self, plotTitle='Live Plot', labelX='t [s]', labelY='data [?]'):
        title(plotTitle)
        xlabel(labelX);
        ylabel(labelY);
        plt.ion()            
        plt.show();
    
    def addColumnToPlot(self, name, columnID, color):
        # Extend lists
        self.columnIDs.append(columnID);
        self.lines.append(plt.plot([], [], c=color, label=name)[0])
        plt.legend()
     
    def refresh(self):
        figure(self.figureId)
        stepSize = floor(self.td.length()/self.maxPoints)+1;
        for i in xrange(0,len(self.columnIDs)):
            self.lines[i].set_xdata(self.td.col(0)[1::stepSize])
            self.lines[i].set_ydata(self.td.col(self.columnIDs[i])[1::stepSize])
        plt.gca().relim()
        plt.gca().autoscale_view(True,True,True)
        plt.draw();
