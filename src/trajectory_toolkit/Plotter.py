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
    legendLoc = 1
    
    def __init__(self, figureId, subplotDim, title='', xLabels = None, yLabels = None, maxPoints=1000):
        self.maxPoints = maxPoints
        self.subplotDim = subplotDim
        if(figureId == -1):
            self.figureId = figure().number
        else:
            self.figureId = figureId
            figure(self.figureId)
        plt.ion()            
        plt.show(block=False);
        plt.gcf().subplots_adjust(bottom=0.15)
        self.yLabels = yLabels
        self.xLabels = xLabels
        if(self.yLabels == None):
            self.yLabels = []
            for i in range(subplotDim[0]*subplotDim[1]):
                self.yLabels.append('')
        if(self.xLabels == None):
            self.xLabels = []
            for i in range(subplotDim[0]*subplotDim[1]):
                self.xLabels.append('')
        axis = subplot(self.subplotDim[0], self.subplotDim[1], 1)
        plt.title(title)
                
    
    def addDataToSubplot(self, td, colID, plotID, formatstring, legend=''):
        self.colIDs.append(colID);
        self.td.append(td)
        # Add lines to subplot
        figure(self.figureId)
        axis = subplot(self.subplotDim[0], self.subplotDim[1], plotID)
        axis.set_xlabel(self.xLabels[plotID-1])
        axis.set_ylabel(self.yLabels[plotID-1])
        self.axes.append(axis);
        self.lines.append(axis.plot([], [], formatstring, label=legend)[0])
        if legend != '':
            plt.legend(loc=self.legendLoc)
        self.refreshSingleLine(len(self.colIDs)-1)
        
    def addDataToSubplotMultiple(self, td, col, plotID, formatstring, legend):
        colIDs = td.getColIDs(col)
        for i in xrange(0,len(colIDs)):
            self.addDataToSubplot(td,colIDs[i],plotID[i],formatstring[i],legend[i])
        
    def refreshSingleLine(self,lineID):
        figure(self.figureId)
        stepSize = floor(self.td[lineID].end()/self.maxPoints)+1;
        self.lines[lineID].set_xdata(self.td[lineID].col(0)[0::stepSize])
        self.lines[lineID].set_ydata(self.td[lineID].col(self.colIDs[lineID])[0::stepSize])
        self.axes[lineID].relim()
        self.axes[lineID].autoscale_view(True,True,True)
        plt.draw()
    
    def setAxis(self, plotID, t0 = None, t1 = None, y0 = None, y1 = None):
        figure(self.figureId)
        axis = subplot(self.subplotDim[0], self.subplotDim[1], plotID)
        xmin, xmax, ymin, ymax = axis.axis()
        if t0==None:
            t0 = xmin
        if t0==None:
            t1 = xmax
        if t0==None:
            y0 = ymin
        if t0==None:
            y1 = ymax
        axis.axis([t0,t1,y0,y1])
        plt.draw()
    
    def setFigureSize(self,w = None,h = None):
        fig = figure(self.figureId)
        size_old = fig.get_size_inches()
        print(size_old)
        if w==None:
            w = size_old[0]
        if h==None:
            h = size_old[1]
        fig.set_size_inches(w, h, forward=True)
    
    def removeTicks(self, plotID):
        figure(self.figureId)
        axis = subplot(self.subplotDim[0], self.subplotDim[1], plotID)
        axis.set_xticks([])
        
    def refresh(self): # More efficient than refreshSingleLine
        figure(self.figureId)
        for i in xrange(0,len(self.colIDs)):
            stepSize = floor(self.td[i].end()/self.maxPoints)+1;
            self.lines[i].set_xdata(self.td[i].col(0)[0::stepSize])
            self.lines[i].set_ydata(self.td[i].col(self.colIDs[i])[0::stepSize])
            self.axes[i].relim()
            self.axes[i].autoscale_view(True,True,True)
        plt.draw()