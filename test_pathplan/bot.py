import numpy as np
from optimizer import Optimizer, Obstacle


class Bot:
    def __init__(self, x, y, theta, v, w):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.w = w
        self.prev_heading_error = 0.0
        self.total_heading_error = 0.0
        self.dt = 0.25
        self.allObstacles: list[Obstacle] = []

    def set_goal(self, goal):
        self.goal = goal

    def move(self):

        if np.sqrt(((self.goal[0] - self.x) ** 2) + ((self.goal[1] - self.y) ** 2)) >= 1:
            self.control()
        else:
            self.v = 0
            self.w = 0

        self.theta = self.theta - self.w * self.dt
        self.x = self.x + self.v * self.dt * np.cos(self.theta)
        self.y = self.y + self.v * self.dt * np.sin(self.theta)

    def control(self):
        Kp = 0.656
        Kd = 0.01
        Ki = 0.0

        delta_theta = (np.arctan2((self.goal[1] - self.y), (self.goal[0] - self.x))) - self.theta

        #delta_theta = ((delta_theta + np.pi) % (2.0 * np.pi)) - np.pi
        #print(str(np.arctan2((self.goal[1] - self.y), (self.goal[0] - self.x)))+" : "+str(self.theta))
        e_new = -delta_theta
        e_dot = (e_new - self.prev_heading_error) / self.dt
        total_heading_error = (self.total_heading_error + e_new) * self.dt

        W = (Kp * e_new) + (Ki * total_heading_error) + (Kd * e_dot)
        self.prev_heading_error = e_new

        d = np.sqrt(((self.goal[0] - self.x) ** 2) + ((self.goal[1] - self.y) ** 2))
        distThresh = 0.1

        V = 1 * (np.arctan(d - distThresh))
        #self.v = V
        #self.w = W
        opt = Optimizer(self.x, self.y, self.theta, V, W)
        for obs in self.allObstacles:
            opt.get_obstacle(obs)
        self.v, self.w = opt.optimize()

