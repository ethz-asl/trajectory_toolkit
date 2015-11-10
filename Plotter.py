from pylab import *
import matplotlib.pyplot as plt

class Plotter:
    t = 0
    x = 0
    tMax = 0
    tMin = 0
    tStart = 0
    xMax = 0
    xMin = 0
    gotData = False
    gotNew = False
    line = []
    figureId = 0
    def __init__(self, figureId):
        self.figureId = figureId
        figure(self.figureId)
        self.line, = plt.plot([], [])
        title('Test Figure')
        ylabel('x')
        xlabel('t')
        grid(b=1)
    def callback(self,data):
        self.t = data.header.stamp.to_sec()
        self.x = data.transform.translation.x
        if(not self.gotData):
            self.tMax = self.t+0.1
            self.tMin = self.t
            self.tStart = self.t
            self.xMax = self.x+0.1
            self.xMin = self.x
        else:
            if(self.tMax < self.t): self.tMax = self.t
            if(self.tMin > self.t): self.tMin = self.t
            if(self.xMax < self.x): self.xMax = self.x
            if(self.xMin > self.x): self.xMin = self.x
        self.gotData = True
        self.gotNew = True
    def refresh(self):
#       ion()
        if(self.gotNew):
            figure(self.figureId)
            self.line.set_xdata(np.append(self.line.get_xdata(), self.t-self.tStart))
            self.line.set_ydata(np.append(self.line.get_ydata(), self.x))
            axis([self.tMin-self.tStart, self.tMax-self.tStart, self.xMin, self.xMax])
            draw()
            show(block=False)
    #       ioff()
            self.gotNew = False
if __name__ == '__main__':
    matplotlib.pyplot.close('all')
    plotter = Plotter()
    plotter.addPlotterToTopic("/vicon/firefly_sbx/firefly_sbx")
