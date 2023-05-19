import numpy as np

class Obstacle(object):
    """
        Class to define Obstacle properties
    """
    def __init__(self, ld, ru):
        """ Init function """

        # Left-down coordinates
        self.ld = ld 

        # Right-up coordinates
        self.ru = ru 
        self.center = [(ld[0] + ru[0]) / 2, (ld[1] + ru[1]) / 2] 

        self_ru = np.array(self.ru)
        self_ld = np.array(self.ld)
        self.radius = (self_ru - self_ld) // 2


