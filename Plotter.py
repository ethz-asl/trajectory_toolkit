from pylab import *
import matplotlib.pyplot as plt
import numpy as np


class Plotter:
    td = []
    lines = []
    axes = []
    colIDs = []
    noSubplots = 1
    figureId = 0
    maxPoints = 1000;
    
    def __init__(self, figureId, noSubplots, isLive=False, maxPoints=1000):
        self.figureId = figureId
        self.maxPoints = maxPoints
        self.noSubplots = noSubplots
        if isLive:
            figure(self.figureId)
            plt.ion()            
            plt.show();
    
    def addDataToAxis(self, td, colID, axis, color, name):
        # Extend lists
        self.colIDs.append(colID);
        self.axes.append(axis);
        self.td.append(td)
        self.lines.append(axis.plot([], [], c=color, label=name)[0])
        plt.legend()
    
    def addDataToSubplot(self, td, colID, subPlotIdx, color, name):
        # Add plots
        figure(self.figureId)
        axis = subplot(self.noSubplots, subPlotIdx[0], subPlotIdx[1])
        self.addDataToAxis(td, colID, axis, color, name)
        
    def refresh(self):
        figure(self.figureId)
        for i in xrange(0,len(self.colIDs)):
            # Subsample Data
            stepSize = floor(self.td[i].length()/self.maxPoints)+1;
            self.lines[i].set_xdata(self.td[i].col(0)[1::stepSize])
            self.lines[i].set_ydata(self.td[i].col(self.colIDs[i])[1::stepSize])
        # Update axis limits
        for axis in self.axes:
            axis.relim()
            axis.autoscale_view(True,True,True)
        # Redraw Plot
        plt.draw();
        
    def show(self):
        # Refresh and show plot
        plt.ioff();            
        self.refresh();
        plt.show();
