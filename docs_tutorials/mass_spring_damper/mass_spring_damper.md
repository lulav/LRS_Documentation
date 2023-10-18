---
sidebar_position: 30
sidebar_label: 'Mass Spring Damper'

---

# Mass Spring Damper

This is a simple ROS demonstration of a mass-spring-damper system.

![jpeg](img/system.jpeg)

The example contains two ROS 2 packages: `dynamics` and `controller`.

## Table of Contents
- [Mass Spring Damper](#mass-spring-damper)
  - [Table of Contents](#table-of-contents)
  - [System Dynamics](#system-dynamics)
  - [The Controller](#the-controller)
  - [CITROS Usage](#citros-usage)
    - [CITROS Installation 🛫](#citros-installation-)
    - [Configuring the project](#configuring-the-project)
    - [Parameters](#parameters)
    - [Launch Files](#launch-files)
    - [Initialize CITROS:](#initialize-citros)
    - [Run with CITROS:](#run-with-citros)
    - [Syncing the Project's setup](#syncing-the-projects-setup)
    - [Normal distributed mass](#normal-distributed-mass)

## System Dynamics

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

## The Controller

You can write your own controller to try stabilize the system for a given setpoint.

the default controller is a simple PID controller with the following form:

$$
        f(t) = {k_pe(t) + k_i\int{e(t)dt}} + k_d {d\over dt}(e(t))
$$

you can tune the controller gains, $k_p$, $k_i$, $k_d$, configured as ROS 2 parameters.

## CITROS Usage

### CITROS Installation 🛫
First of all, to use all the powerfull CITROS features usage requires CITROS installation: follow the instructions on the CITROS CLI [documentation](https://citros.io/doc/docs/cli/cli_install)

### Configuring the project

After all the prerequisites done, we can start configuring our project. Open the project's repository in `VSCode` and reopen the project in a the VSCode's `devcontainer`.

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
|x | dynamics | The mass initial poisition
|v | dynamics | The mass initial velocity
|a | dynamics | The mass initial acceleration

### Launch Files

|Launch file| Description
| --------|  --------|
|dynamics.launch.py | Launch the uncontrolled system
|dynamics_controller.launch.py | Launch the controlled system with PID controller

### Initialize CITROS:

        citros init

### Run with CITROS:

        citros run -n "default" -m "default simulation"

After running the command, choose the launch file you wish to run. The simulation will start and you could see the mass position and the control signal in the terminal's logs.

### Syncing the Project's setup

Follow the [guide](https://citros.io/doc/docs/cli/cli_commands/cli_sync) for syncing CITROS project with the server.



### Normal distributed mass

Supposed we tune the PID gains of the controller for the nominal mass, and we reached a satisfying results.

Now we want to know how robust was the tuning for a normal distributd mass:

$$ 
m = N(\mu, \sigma)
$$

where:

$$ 
\mu = 1.0,    
\sigma = 0.3 
$$

All the parameters can be set following the CITROS parameter guide.

**TODO: add parameter guide**

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

Lets run `50` simulations in the server and analyze the results:

![jpeg](img/analysis.jpeg)


We can see that `43` tests were passed the requirements, `6` failed and `1` was generated invalid mass $(<0)$.

The maximum mass that still meeting the requirements is equal to $1.335 [kg]$


Can you do better?