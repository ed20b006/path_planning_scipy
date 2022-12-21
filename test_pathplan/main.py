import time

import matplotlib.pyplot as plt
from bot import Bot
from optimizer import Obstacle

mybot = Bot(25, 25, 0, 0, 0)
source = [60, 75]
obstacles = [
    Obstacle(40, 35, 0, 0, 0),
    Obstacle(50, 60, 0, 0, 0),
    Obstacle(35, 50, 0, 0, 0),
    Obstacle(70, 50, 0, 0, 0)
]
mybot.set_goal(source)

plt.ion()

while True:
    #plt.clf()
    plt.xlim([0, 100])
    plt.ylim([0, 100])
    plt.plot(mybot.x, mybot.y, 'bo', markersize = 3)
    plt.plot(mybot.goal[0], mybot.goal[1], 'go', markersize = 3)
    for obs in obstacles:
        plt.plot(obs.x, obs.y, 'ro', markersize = 3)
    plt.draw()
    mybot.allObstacles = obstacles
    mybot.move()
    plt.pause(0.025)
