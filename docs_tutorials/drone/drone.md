---
slug: Drone
title: Drone
authors: [iftahnaf]
tags: [CITROS]
---

# Drone Tutorial

![Alt text](img/Droneimg.png)

**Table of contents**

1. [General Information](#general-Information-🌐)
2. [Installation](#installation-🛫)
3. [CITROS Integration](#citros-integration-🛸)
    1. [Configuring The Project](#configuring-the-project-⚙️)
    2. [parameters](#parameters-🐄)  
4. [Example](#example-🕹️)  
    1. [The Scenario](#the-scenario-🎥)  
    2. [Run A Test Simulation In The Web With CITROS](#run-a-test-simulation-in-the-web-with-citros-📡)

## General Information 🌍

This repository contains an example of a ROS 2 node which communicate with a drone using PX4 and ROS 2.
Communication via uXRCE-DDS (PX4 version `1.14.0`).  
The repository contains launch file which launch:
1. Gazebo simulation (headless / gui options available).
2. A PX4 instans which control the simulated drone.
3. DDS agent for ROS 2 - PX4 communication.
4. An Offboard node which sends setpoints for the control system.

## Installation 🛫

Clone the repository from Github:

        https://github.com/citros-garden/drone.git

Then open the repository in VSCode's `devcontainer` with `reopen in container option`.
The [Dockerfile](.devcontainer/Dockerfile) contains all the necessary dependencies for the project, and the 
[install](.devcontainer/install.sh) script will clone PX4-Autopilot and build the firmware, along with building ROS 2 workspace.

## CITROS Integration 🛸
To use all the powerfull CITROS features usage requires CITROS installation:  

**First,reopen the folder localy** and then follow the instructions:

        pip install citros


then login:

        citros login 



### Configuring The Project ⚙️

After all the prerequisites done, we can start configuring our project. 

1.  Initialize CITROS:

        citros init

Now you can see ```.citros``` folder in the explorer.  

2.  Configuring the params setup.  
You can find the default setup in ```.citros/parameter_setups/default_param_setup.json```.  
[CITROS CLI](https://citros.io/doc/docs_citros_web/repos/repos_file_structure/repos_fs_param_setup) provides more information about how to change the parameters by the user.
### Parameters 🐄	
This is a list of all the ROS 2 parameters that can be controll by the user wish:

|     Variable     | Description | package |
| -------- |    ------- |  ------- | 
| MC_PITCHRATE_D | pitch rate d | px4_config |
| MC_PITCHRATE_I | pitch rate i |  px4_config |
| MC_PITCHRATE_K | pitch rate k |  px4_config |
| MC_PITCHRATE_P | pitch rate p |  px4_config |
| MC_PITCH_P | pitch p |  px4_config |
| MC_ROLLRATE_D | roll rate d |  px4_config |
| MC_ROLLRATE_I | roll rate i |  px4_config |
| MC_ROLLRATE_K | roll rate k |  px4_config |
| MC_ROLLRATE_P | roll rate p |  px4_config |
| MC_ROLL_P | roll p |  px4_config |
| p1_x , p1_y, p1_z | position [m] of point 1, NED|  px4_offboard |
| p2_x , p2_y, p2_z | position [m] of point 2, NED|  px4_offboard |
| p3_x , p3_y, p3_z | position [m] of point 3, NED|  px4_offboard |
| p4_x , p4_y, p4_z | position [m] of point 4, NED|  px4_offboard |
| repeats | number of repeats|  px4_offboard |
| tolerance | tolerance to destination|  px4_offboard |
| timer_period | [sec] offboard timer |  px4_offboard |
| ixx | inertia moment at x derication |  rigid_body |
| iyy | inertia moment at y derication |  rigid_body |
| izz | inertia moment at z derication |  rigid_body |
| ixy | inertia moment at xy derication |  rigid_body |
| ixz | inertia moment at xz derication |  rigid_body |
| iyz | inertia moment at yz derication |  rigid_body |
| mass | total mass of the drone |  rigid_body |
| gyroscopeNoiseDensity | Gyroscope noise density (two-sided spectrum) [rad/s/sqrt(Hz)] |  sensors |
| gyroscopeRandomWalk | Gyroscope bias random walk [rad/s/s/sqrt(Hz)] |  sensors |
| gyroscopeBiasCorrelationTime |Gyroscope bias correlation time constant [s] |  sensors |
| gyroscopeTurnOnBiasSigma |Gyroscope turn on bias standard deviation [rad/s] |  sensors |
| accelerometerNoiseDensity |Accelerometer noise density (two-sided spectrum) [m/s^2/sqrt(Hz)] |  sensors |
| accelerometerRandomWalk |Accelerometer bias random walk [m/s^2/s/sqrt(Hz)] |  sensors |
| accelerometerBiasCorrelationTime |Accelerometer bias correlation time constant [s] |  sensors |
| accelerometerTurnOnBiasSigma |Accelerometer turn on bias standard deviation [m/s^2] |  sensors |
| windVelocityMean |the mean velocity of the wind |  world |
| windVelocityMax |the max velocity of the wind |  world |
| windVelocityVariance |the velocity variance of the wind |  world |
| windDirectionMean |the mean direction of the wind |  world |
| windDirectionVariance |the direction variance of the wind |  world |
| windGustStart | |  world |
| windGustDuration | |  world |
| windGustVelocityMean | |  world |
| windGustVelocityMax | |  world |
| windGustVelocityVariance | |  world |
| windGustDirectionMean | |  world |
| windGustDirectionVariance | |  world |


After setting the preferred parameters, save the file and verify the settings using the CLI command: `CITROS status`.
Now we can run a simulation.

## Example 🕹️
### **The Scenario** 🎥
Let's repeat hovering the drone between points 1 and 4 five times while keeping all parameters default except for the wind velocity mean, which changes uniformly between 0 and 20.  
You can see the paramaters file [here](https://citros.io/drone/blob/main/parameter_setups/wind.json).

### Run A Test Simulation In The Web With CITROS 📡

When we finished setting up the parameters file we want to run it with CITROS.

- First, we save our work and uploud it to CITROS server:
```bash 
 citros commit
 citros push
```

- Then,we need to build and push Docker container image to the CITROS server:
```bash 
citros docker-build-push
```

- Finely, we run the simulation at the web:
1.  Go to the ```Repositories``` page clicking on the tab on the top;
2.  Find the drone project and open it;
3. Navigate to the ```Runs``` tab;
4. Click on the ```Run Simulation``` button on the right;

For our example , we want to run [the scenario](#the-scenario-🎥) 50 times.  
When the simulations are finished we can create a notebook file, you can find our case [here](https://citros.io/drone/blob/main/notebooks/wind_analysis.ipynb).  
The results were:  

![Alt text](img/wind_sim_result.png)
