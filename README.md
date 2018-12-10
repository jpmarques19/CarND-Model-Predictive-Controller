# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Overview

This project implements an advanced type of controller, called **Model Predictive Control (MPC)** that works by  
successively calculating optimal trajectories and returning the necessary actuation controls to follow those trajectories at each timestep. 

An optimization algorithm called **Ipopt** and an automatic differentiation library **CppAD** are used in the project.


## The Model

The cinematic model used follows this set of equations:

x[t1] = x[t0] + v[t0] * cos(psi[t0]) * dt

y[t1] = y[t0] + v[t0] * sin(psi[t0]) * dt

psi[t1] = psi[t0] + v[t0] / Lf * delta[t0] * dt

v[t1] = v[t0] + a[t0] * dt

cte[t1] = (y[t0] - f(x[t0])) + (v[t0] * sin(epsi[t0]) * dt)

epsi[t1] = (psi[t0] - psides[t0]) + (v[t0] * delta[t0] / Lf * dt)

Where:

**x,y** - Position coordinates  
**psi** - Heading  
**v** - Velocity  
**cte** - Crosstrack error  
**epsi** - Heading error 
**delta** - Steering angle
**a** - Throtlle
**Lf** - Model constant (provided and tested by udacity)

**State variables** = [x, y, psi, v, cte, epsi]
**Actuators** = [delta, a]


## Timestep Length and Elapsed Duration (N and dt)

To choose these parameters I first defined a sensible time horizon T. In the case of real driving this value should  
be a few seconds at most. I went with T = 1, although larger values can result in better predictions.  

I then experimented with different values of N = {5,10,20} and corresponding dt = {0.2,0.1,0.05}.

The first case (N=5,dt=0.2) has very low resolution leading to very unpredictable results and   
the third case (N=20,dt=0.05) is prone to latency due to excessive computation time. 

Since the second case (N=10,dt=0.1) delivered satisfactory results, this was my final choice. 

## Polynomial Fitting and MPC Preprocessing

The waypoints provided by the simulator are transformed to the car coordinate system at ./src/main.cpp from line 96 to line 105. Then a 3rd-degree polynomial is fitted to the transformed waypoints. These polynomial coefficients are used to calculate the cte and epsi later on. They are used by the solver as well to create a reference trajectory.

## Model Predictive Control with Latency
To handle actuator latency, the state values are calculated using the model and the delay interval. These values are used instead of the initial one. The code implementing that could be found at ./src/main.cpp from line 116 to line 134.