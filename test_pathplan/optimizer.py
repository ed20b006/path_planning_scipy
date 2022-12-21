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
        self.depth = 5
        self.x = np.zeros(2 * self.depth)
        self.X = X
        self.Y = Y
        self.theta = theta
        self.V = V
        self.W = W
        self.obstacles: list[Obstacle] = []
        self.obsD = 4.0
        self.alpha = 2.0
        self.beta = 100.0
        self.gama = 0.5

    def heading(self, v, w):
        head = 0
        for i in range(self.depth):
            head += abs(self.V - v[i]) + abs(self.W - w[i])

        return head

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
        dis = 0

        X1 = self.X
        Y1 = self.Y
        THETA1 = self.theta

        x = [obs.x for obs in self.obstacles]
        y = [obs.y for obs in self.obstacles]
        theta = [obs.theta for obs in self.obstacles]

        for i in range(self.depth):
            X1, Y1, THETA1 = self.cal_next_pos(X1, Y1, THETA1, v[i], w[i], dt)

            for j, obs in enumerate(self.obstacles):
                x[j], y[j], theta[j] = self.cal_next_pos(x[j], y[j], theta[j], obs.v, obs.w, dt)
                dis += self.cal_distance(x[j], y[j], X1, Y1)

        return dis

    def get_obstacle(self, obs: Obstacle):
        if self.cal_distance(obs.x, obs.y, self.X, self.Y) < self.obsD:
            self.obstacles.append(obs)

    def const_vel(self, v, w):
        err = 0
        for i in range(self.depth - 1):
            err += abs(v[i] - v[i + 1]) + abs(w[i] - w[i + 1])

        return err

    def objective(self, x):
        v = [x[i] for i in range(len(x)) if i % 2 == 0]
        w = [x[i] for i in range(len(x)) if i % 2 != 0]
        G = self.alpha * self.heading(v, w) - self.beta * self.dis(v, w) + self.gama * self.const_vel(v, w)
        return G

    def optimize(self):
        b = ((0.5, 3), (-1, 1))
        bnds = b * self.depth
        solution = minimize(self.objective, self.x, method='SLSQP', bounds=bnds)
        #print(f"{solution.x[0]:.2f} : {solution.x[1]:.2f}")
        #print(self.obstacles)
        return solution.x[0], solution.x[1]
