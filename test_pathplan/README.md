## main.py
* Plots bot, goal and obstacle every 0.01 sec
* Calls Bot.move every frame

## bot.py

### Bot
* Initializes position, orientation, velocity, angular velocity
* _dt_ = 0.25

#### set_goal()
Sets goal co-ordinates

#### move()
* Calls *control()* if bot is far away from goal
* Sets _x_, _y_, _theta_ every _dt_

#### control()
* Simulates PID controller and sets the _v_, _w_ accordingly
* Further optimiziation using Dynamic Window Path Plannning using *Optimizer*

## optimizer.py

### Obstacle
Stores _x_, _y_, _theta_, _v_, _w_ of each obstacle

### Optimizer
* gets current _x_, _y_, _theta_ and _V_ & _W_ calculated using PID controller

#### heading()
* `head` calculates error of optimization parameters(_x_) from _V_ and _W_
* current code assumes best _W_ for future points to be the same value as _W_ calculated for next point but can be solved solved by dividing it by _self.depth_ (downside of this is showN as GIF **REF 1 & 2**)

#### cal_distance()
* gets co-ordinate of two points
* returns distance between these points

#### cal_next_pos()
* given position data and time --> returns next positions co-rodinate

#### dis()
* `dis` calculates distance from obstacles near the robot
NOTE :`self.cal_distance(x[j], y[j], X1, Y1) ** 2` tends to reduce _self.beta_ exponentially

#### get_obstacle()
* gets obstacles near the bot

#### const_vel()
* `const_vel` calculates jerk and angular jerk as much as possible

#### objective()
* `objective` calculates `min alpha * head - beta * dis - gama * const_vel`
  * minmizes `head`
  * maximizes `dis`
  * minimizes `const_vel`

#### optimize()
* sets boundary points for _v_ and _w_
* minimizes `objective` with _v_ and _w_ of next `self.depth` number of points and uses **SLSQP** 
* returns only the next _v_ and _w_

