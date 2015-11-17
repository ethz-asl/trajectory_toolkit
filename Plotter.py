from pylab import *
import matplotlib.pyplot as plt
import numpy as np


class Plotter:
    td = []
    colIDs = []
    lines = []
    axes = []
    subplotDim = [1,1]
    figureId = 0
    maxPoints = 1000;
    
    def __init__(self, figureId, subplotDim, maxPoints=1000):
        self.figureId = figureId
        self.maxPoints = maxPoints
        self.subplotDim = subplotDim
        figure(self.figureId)
        plt.ion()            
        plt.show(block=False);
    
    def addDataToSubplot(self, td, colID, plotID, formatstring, legend=''):
        self.colIDs.append(colID);
        self.td.append(td)
        # Add lines to subplot
        figure(self.figureId)
        axis = subplot(self.subplotDim[0], self.subplotDim[1], plotID)
        self.axes.append(axis);
        self.lines.append(axis.plot([], [], formatstring, label=legend)[0])
        if legend != '':
            plt.legend()
        self.refreshSingleLine(len(self.colIDs)-1)
        
    def addDataToSubplotMultiple(self, td, colID, plotID, formatstring, legend):
        for i in xrange(0,len(colID)):
            self.colIDs.append(colID[i]);
            self.td.append(td)
            # Add lines to subplot
            figure(self.figureId)
            axis = subplot(self.subplotDim[0], self.subplotDim[1], plotID[i])
            self.axes.append(axis);
            self.lines.append(axis.plot([], [], formatstring[i], label=legend[i])[0])
            if legend[i] != '':
                plt.legend()
            self.refreshSingleLine(len(self.colIDs)-1)
        
    def refreshSingleLine(self,lineID):
        figure(self.figureId)
        stepSize = floor(self.td[lineID].end()/self.maxPoints)+1;
        self.lines[lineID].set_xdata(self.td[lineID].col(0)[0::stepSize])
        self.lines[lineID].set_ydata(self.td[lineID].col(self.colIDs[lineID])[0::stepSize])
        self.axes[lineID].relim()
        self.axes[lineID].autoscale_view(True,True,True)
        plt.draw()
        
    def refresh(self):
        figure(self.figureId)
        for i in xrange(0,len(self.colIDs)):
            # Subsample Data
            stepSize = floor(self.td[i].end()/self.maxPoints)+1;
            self.lines[i].set_xdata(self.td[i].col(0)[0::stepSize])
            self.lines[i].set_ydata(self.td[i].col(self.colIDs[i])[0::stepSize])
        # Update axis limits
        for axis in self.axes:
            axis.relim()
            axis.autoscale_view(True,True,True)
        # Redraw Plot
        plt.draw()