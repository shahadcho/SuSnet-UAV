
import numpy as np


class Intruder:
    def __init__(self, center, radius, velocity, xlim, ylim,):
        self.center = center
        self.radius = radius
        self.velocity = velocity
        self.xlim = xlim
        self.ylim = ylim
    def move(self, dt, obstacles):
        # update velocity randomly
        noise = np.random.normal(0, 0.1, 2)
        self.velocity += noise
        # normalize velocity to have constant speed
        self.velocity = (np.array(self.velocity) / np.linalg.norm(self.velocity))*0.5
        
        # check for obstacle collision and avoid them
        for obs in obstacles:
            self_center = np.array(self.center)
            obs_center = np.array(obs.center)
            dist = np.linalg.norm(self_center - obs_center)
            if dist < self.radius + obs.radius.all():
                # calculate the direction away from the obstacle
                self_center=np.array(self.center)
                obs_center=np.array(obs.center)
                away_dir = (self_center - obs_center) / dist
                # adjust velocity away from obstacle
                self.velocity += away_dir
                break
                
        # update position
        self.center = [c + v*dt for c, v in zip(self.center, self.velocity)]
        
        # check for boundary collision and adjust velocity if needed
        if self.center[0] < self.xlim[0] + self.radius:
            self.velocity[0] = abs(self.velocity[0])
        elif self.center[0] > self.xlim[1] - self.radius:
            self.velocity[0] = -abs(self.velocity[0])
        if self.center[1] < self.ylim[0] + self.radius:
            self.velocity[1] = abs(self.velocity[1])
        elif self.center[1] > self.ylim[1] - self.radius:
            self.velocity[1] = -abs(self.velocity[1])
            
        # update position with new velocity
        self.center = [c + v*dt for c, v in zip(self.center, self.velocity)]
        
