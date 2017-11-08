# Self Driving Nanodegree Model Predictive Controller

---

The goal of this project is to implement model predictive control to drive the vehicle around the track as close as reference trajectory with additional latency to simulate real-world scenario.

[//]: # (Image References)
[image1]: ./max_speed.png "Car with maximum speed in simulator"

**The Model**

In order to determine next state (state at t+1) from our state vector at t and our actuators values (delta, a) following model is used.

```
x1 = x + v * cos(psi) * dt
y1 = y + v * sin(psi) * dt
psi1 = psi + v / Lf * delta * dt
v1 = v + a * dt
```
Where x, y are vehicle coordinates at time t, psi and v are the vehicle orientation and velocity respectively at time t.

The main purpose is to minimize the difference between reference trajectory and vehicle's actual path. So in order to minimize the error, we compute it using equations as shown below and then adjust the control input accordingly.

```
cte1 = cte0 + vt * sin(e_psi) * dt where cte0 = f(x) - y
e_psi = psi - arctan(f'(x)) + v / Lf * delta * dt 
```

Here, cte is the cross track error which is the difference between the reference trajectory and the current vehicle position y. And e_psi is the orientation Error.


**Timestep Length and Elapsed Duration (N & dt)**

N is the number of timesteps on the horizon. dt is how much time elapses between actuations. For example, if N were 25 and dt were 0.4, then T would be 10 seconds.

MPC attempts to approximate a continuous reference trajectory by means of discrete paths between actuations. Larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. This is sometimes called "discretization error".

Also, our T (N*dt) should not be too large as well. Because for large values the environment will change enough that it won't make sense to predict any further into the future.

The values I chose for N and dt are `10` and `0.1`. These values are taken from project walk-through video of MPC-Controller by Aron Brown and Dominique Luna.
Adjusting either N or dt produced erratic behavior sometimes. Other values I tried are 8/0.125, 12/0.1, 15/0.05, 25/0.04.

With the value of N increased, computation gets increased for each of model elements. The length of the timestamps is chosen small with large N which provides good accuracy. However, due to large computation, it adds latency in actuation. That made vehicle to steer left and right at very short intervals. As a result, the vehicle just lost the track and stopped moving in few seconds.


**Polynomial Fitting and MPC Preprocessing**

The waypoints shown in yellow trajectory are first transformed into vehicle coordinate system first. The waypoints are obtained by shifting the origin to the current position of the vehicle and then fitted in third order polynomial. This transformation can be found in main.cpp (107 - 114); 

    `state << 0, 0, 0, v, cte, epsi;`

The initial position of the car and heading direction are always zero in vehicle coordinate system. Hence, the state of the car in this system is as shown above initially.

**Model Predictive Control with Latency**

In order to cover the realistic delay of command propagating through the system, 100 milliseconds delay is added to each actuation command.

I found this approach to implement latency in the project through a slack channel.

```
double latency = 0.1;
px = px + v*cos(psi)*latency;
py = py + v*sin(psi)*latency;
psi = psi - v*steer_value/Lf*latency;
v = v + throttle_value*latency;
```

As we can see in the above equations that dt is replaced by latency, rest of all is similar to state equations of the model.


### Fine tuning of error multiplication factor: ###

**Final values after many trial and errors**

```
ref_v = 80
Cross-track Error Factor = 1500
Orientation Error Factor = 1500
Velocity Error Factor = 1
Delta Actuation Factor = 5
Acceleration Actuation Factor = 5
Delta Actuation Change Factor = 500
Acceleration Actuation Change Factor = 10
```

With hyperparameters as explained above, I was able to get ~76.50 mph speed highest as shown below.

![alt text][image1]