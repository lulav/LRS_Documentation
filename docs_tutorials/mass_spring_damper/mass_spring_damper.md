---
sidebar_position: 30
sidebar_label: 'Mass Spring Damper'

---

## Overview

This is a simple ROS demonstration of a mass-spring-damper system.

![jpeg](img/system.jpeg)

The example contains two ROS 2 packages: `dynamics` and `controller`.

### System Dynamics

The system's equations of motion:

$$
        m\ddot x =  kf(t) -c\dot x -kx
$$

and after laplace transformation (with zero I.C) we get a second order system:

$$
        {X \over F} = {\omega_n^2 \over s^2 +2\omega_n\zeta s + \omega_n^2}
$$

where the natural frequency $\omega_n = \sqrt{k \over m}$

You can choose the system's parameters `m`, `k` and `c` and choose the initial condition `x0`, `v0` and `a0`, all configured as ROS 2 parameters.

### The Controller

You can write your own controller to try stabilize the system for a given setpoint.

the default controller is a simple PID controller with the following form:

$$
        f(t) = {k_pe(t) + k_i\int{e(t)dt}} + k_d {d\over dt}(e(t))
$$

you can tune the controller gains, $k_p$, $k_i$, $k_d$, configured as ROS 2 parameters.

## Prerequisites
Make sure you complete the [Getting Started Tutorial](https://citros.io/doc/docs_tutorials/getting_started/).
For working without dockers **(not recommended)**, please check the [.devcontainer](https://github.com/citros-garden/mass-spring-damper/tree/main/.devcontainer) folder in the project's repo for dependencies (in the `Dockerfile` and `install.sh`).

## Table of Contents
- [Installation](#installation)
- [Workspace Overview](#workspace-overview)
- [CITROS Initialization](#citros-initialization)
- [Scenario](#scenario)
- [Running the scenario using CITROS](#running-the-scenario-using-citros)
- [Results](#results)

## Installation
Clone the repository from Github:
```sh
git clone git@github.com:citros-garden/mass-spring-damper.git
```
Then open the repository in VSCode's `devcontainer` with `reopen in container option`.  

The Dockerfile contains all the necessary dependencies for the project.

## Workspace Overview

### Parameters

| Parameter | Package | Description
| --------|  --------|  --------|
|kp | controller | p gain of the PID controller
|ki | controller | i gain of the PID controller
|kd | controller | d gain of the PID controller
|setpoint | controller | Setpoint position for the controller
|m | dynamics | The mass of the system
|k | dynamics | The spring coefficient
|c | dynamics | The damper coefficient
|x | dynamics | The mass initial position
|v | dynamics | The mass initial velocity
|a | dynamics | The mass initial acceleration

### Launch Files

|Launch file| Description
| --------|  --------|
|dynamics.launch.py | Launch the uncontrolled system
|dynamics_controller.launch.py | Launch the controlled system with PID controller

## CITROS Initialization
Make sure to install and initialize CITROS by following the [Getting Started](https://citros.io/doc/docs_tutorials/getting_started/) tutorial.

## Scenario

Supposed we tune the PID gains of the controller for the nominal mass, and we reached a satisfying results.

Now we want to know how robust was the tuning for a normal distributed mass:

$$ 
m = N(\mu, \sigma)
$$

where:

$$ 
\mu = 1.0,    
\sigma = 0.3 
$$

All the parameters can be set following the CITROS [parameter guide](https://citros.io/doc/docs/repos/repos_file_structure/repos_fs_param_setup).


The initial condition are:

$$
r_0 = -1.0[m]\\
v_0 = 0.0 [m/s]\\
a_0 = 0.0 [m/s^2]\\
$$

With $setpoint = 0.0 [m]$.

We will define the following requirements:

* Maximum overshoot of `30%`.
* Settling time is `2.0` [sec].
* Settling to `10%` of the steady-state value.

## Running the scenario using CITROS
```bash
citros run -n "default" -m "default simulation"
```
After running the command, choose the launch file you wish to run. The simulation will start and you could see the mass position and the control signal in the terminal's logs.

For more CLI running options check the [Introduction to CITROS](https://citros.io/doc/docs_tutorials/) tutorial.

## Results
Lets run `50` simulations in the server and analyze the results:

![jpeg](img/analysis.jpeg)


We can see that `43` tests were passed the requirements, `6` failed and `1` was generated invalid mass $(<0)$.

The maximum mass that still meeting the requirements is equal to $1.335 [kg]$

Can you do better?