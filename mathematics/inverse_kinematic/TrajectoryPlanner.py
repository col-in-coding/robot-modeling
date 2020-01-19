import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

class TrajectoryPlanner:

    def __init__(self):
        self.link = np.array([])
        # angle limits
        self.angleRange1 = np.pi / 2
        self.angleRange2 = np.pi
