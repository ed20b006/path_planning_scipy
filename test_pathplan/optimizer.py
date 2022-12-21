import numpy as np
from scipy.optimize import minimize

dt = 0.25

class Obstacle:

    def __init__(self, x, y, theta, v, w):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.w = w


class Optimizer:

    def __init__(self, X, Y, theta, V, W):
        self.x = np.zeros(2)
        self.x[0] = V
        self.x[1] = W
        self.X = X
        self.Y = Y
        self.theta = theta
        self.V = V
        self.W = W
        self.obstacles: list[Obstacle] = []
        self.obsD = 7.0
        self.alpha = 1.0
        self.beta = 20.0

    def heading(self, v, w):
        return abs(self.V - v) + abs(self.W - w)

    @staticmethod
    def cal_distance(x1, y1, x2, y2):
        return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    @staticmethod
    def cal_next_pos(x0, y0, theta0, v, w, dt):
        theta = theta0 - w * dt
        x = x0 + v * np.cos(theta) * dt
        y = y0 + v * np.sin(theta) * dt
        return x, y, theta

    def dis(self, v, w):
        global dt
        X1, Y1, THETA1 = self.cal_next_pos(self.X, self.Y, self.theta, v, w, dt)
        dis = 0

        for obs in self.obstacles:
            x, y, theta = self.cal_next_pos(obs.x, obs.y, obs.theta, obs.v, obs.w, dt)
            dis += self.cal_distance(x, y, X1, Y1) ** 2

        return dis

    def get_obstacle(self, obs: Obstacle):
        if self.cal_distance(obs.x, obs.y, self.X, self.Y) < self.obsD:
            self.obstacles.append(obs)

    def objective(self, x):
        G = self.alpha * self.heading(x[0], x[1]) - self.beta * self.dis(x[0], x[1])
        return G

    def optimize(self):
        bnds = ((0.5, 2), (-1, 1))
        solution = minimize(self.objective, self.x, method='SLSQP', bounds=bnds)
        print(f"{solution.x[0]:.2f} : {solution.x[1]:.2f}")
        return solution.x[0], solution.x[1]
