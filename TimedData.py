import numpy as np

class TimedData:
    t = []
    d = []
    N = 0
    def __init__(self, N):
        self.N = N