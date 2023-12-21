---
sidebar_position: 40
sidebar_label: 'Robotic Arm'
---


# Robotic Arm

## Overview
This project is designed to wrap the [Robotic Arm Simulation project](https://github.com/dvalenciar/robotic_arm_environment) and integrate it with CITROS platform. The node provides ROS 2 parameters that allow users to adjust target arm joints position. In addition, the Inverse Kinematic ROS 2 node was added.

![png](img/arm0.png "Arm")

## Prerequisites

1. Please make sure you have all the [necessary softwares](https://citros.io/doc/docs_tutorials/getting_started/#softwares-to-work-with-citros) to work with CITROS installed on your computer.
2. Install [Visual Studio code](https://code.visualstudio.com/download).
3. We strongly recommend that you work with [dockers](https://citros.io/doc/docs_tutorials/dockerfile_overview/). However, if you wish to work without dockers, please refer to the .devcontainer [directory](https://github.com/citros-garden/robotic_arm/tree/main/.devcontainer) in project's repo, the dependencies you need are in the ```Dockerfile``` file.
4. (Optional) Install [Foxglove](https://docs.foxglove.dev/docs/introduction).

## Table of Contents
1. [Installation](#installation)
2. [Workspace Overview](#workspace-overview)
3. [CITROS Initialization](#citros-initialization)
4. [Scenario](#scenario)
5. [Running the Scenario Using CITROS](#running-the-scenario-using-citros)
6. [Results](#results)

## Installation
1. Clone the repository:
```bash
git clone git@github.com:citros-garden/robotic_arm.git
```
2. Open the repository in the [VScode Dev Container](https://citros.io/doc/docs_tutorials/getting_started/#open-project-in-vscode-dev-container).

## Workspace Overview

The Robotic Arm simulation has the following ROS 2 parameters:

* Forward Kinematic simulation parameters:

    |Parameter	|Description |Package
    |--|--|--
    j0		|First joint target position 	|my_doosan_pkg
    j1		|Second joint target position  	|my_doosan_pkg
    j2		|Third joint target position  |my_doosan_pkg
    j3		|Fourth joint target position  |my_doosan_pkg
    j4		|Fifth joint target position  |my_doosan_pkg
    j5		|Sixth joint target position  |my_doosan_pkg

* Inverse Kinematic simulation parameters:

    |Parameter	|Description |Package
    |--|--|--
    pos0		|Arm target position by first axis	|inverse_kinematic_pkg
    pos1		|Arm target position by second axis |inverse_kinematic_pkg
    pos2		|Arm target position by third axis |inverse_kinematic_pkg
    ori0		|Arm target orientation by first axis |inverse_kinematic_pkg
    ori1		|Arm target orientation by second axis  |inverse_kinematic_pkg
    ori3		|Arm target orientation by third axis |inverse_kinematic_pkg


This project contains two launch files: 

|Launch File	|Description |Package
|--|--|--
my_doosan_gazebo_controller.launch.py		|Gazebo Robotic Arm launch file for Forward Kinematic |my_doosan_pkg
simulation_inverse_kinematic_pkg.launch.py	|Gazebo Robotic Arm launch file for Inverse Kinematic |inverse_kinematic_pkg


## CITROS Initialization

1. [Install CITROS](https://citros.io/doc/docs_tutorials/getting_started/#installation).
2. Follow [these steps](https://citros.io/doc/docs_tutorials/getting_started/#initialization) to Initialize CITROS.

Now you can see .citros directory in the explorer.

## Scenario
* Inverse kinematics <br/>
    The Inverse Kinematic calculations are used to show more useful way to interact with robotic arm.  <br/>
    User provides target position and target rotation of arm by three axes as input parameters to configure the simulation. The robotic arm will change its position trying to meet the target conditions. 

* Forward kinematics <br/>
    Forward Kinematic calculation is the simplest way to control the robotic arm behavior. <br/> 
    User provides joints target positions as input parameters to configure the simulation. The robotic arm will change its position trying to meet the target conditions, and the simulation publishes the current positions of joints as a result.<br/>
    
    In this tutorial, let's check how the robotic arm's behavior changes depending on target position. <br />
    For that, we will launch a batch simulation with a random distribution ranging between 0.0 and 0.5, for the last joint position parameter  (`j5`).
    The parameter will be set by a NumPy random function and its  setup is listed in ```.citros/parameter_setups/default_param_setup.json```: <br/>

```json
{
    "packages": {
        "my_doosan_pkg": {
            "trajectory_points_act_server": {
                "ros__parameters": {
                    "j0": -0.8,
                    "j1": 0.2,
                    "j2": 0.75,
                    "j3": -0.102,
                    "j4": 1.57,
                    "j5": {
                        "function": "numpy.random.uniform",
                        "args": [0.0, 0.5]
                    }
                }
            }
        }
    }
}
```

Learn more about parameter setup and defining custom functions in [Directory parameter_setups](https://citros.io/doc/docs_cli/structure/citros_structure/#directory-parameter_setups) and [Adding Functions to Parameter Setup](https://citros.io/doc/docs_cli/configuration/config_params) pages.

In addition to parameter setup, you can configure the simulation performance setup (timeout, CPU, GPU and Memory) as well.
These parameters can be found in ```.citros/simulations/simulation_my_doosan_gazebo_controller.json``` for Forward Kinematic or ```.citros/simulations/simulation_inverse_kinematic_pkg.json``` for Inverse Kinematic. <br/>
Look in [Directory simulations page](https://citros.io/doc/docs_cli/structure/citros_structure#directory-simulations) for more information.

## Running the Scenario Using CITROS

### Running Locally

First, we recommend to update the simulation performance timeout to 300 seconds:

 ```json 
{
    ...
    "parameter_setup": "default_param_setup.json",
    "storage_type": "MCAP",
    "timeout": 300
}
 ```

Then, ensure that the project has been [built and sourced](https://citros.io/doc/docs_tutorials/getting_started/#build-the-project).<br/>

Now we can launch it locally:
```bash 
>>> citros run -n 'robotic_arm' -m 'local test run'
? Please choose the simulation you wish to run:
❯ simulation_my_doosan_gazebo_controller
  simulation_inverse_kinematic_pkg
```
Select the ```simulation_my_doosan_gazebo_controller``` launch file and press ```Enter``` button and wait for the output in the terminal.

```bash
created new batch_id: <batch_run / batch name>. Running locally.
+ running batch [<batch_run / batch name>], description: local test run, repeating simulations: [1]
+ + running simulation [0]
...
```

All the results will be saved in ```.citros/runs/[simulation_name]``` folder.

To plot the local run results you can use [Foxglove](https://citros.io/doc/docs_tutorials/#visualization-with-foxglove) with the ```layout.json``` layout that exists in the Foxglove_layouts directory.

![gif](img/foxglove1.gif "Foxglove example")

### Running in Cloud

First, we recommend to update the simulation performance parameters:
- CPU: 4
- GPU: 4  
- Memory: 4096 MB
- Timeout: 300 seconds


```json
{
    "CPU": 4,
    "GPU": 4,
    "MEM": 4096,
    ...
    "timeout": 300
}
```

Then, [Upload project to CITROS Server](https://citros.io/doc/docs_tutorials/getting_started/#upload-to-citros-server). 

Finally, we can run it in the cloud! Simply add `-r` to the terminal command: 
```bash 
citros run -n 'robotic_arm' -m 'cloud test run' -r
? Please choose the simulation you wish to run:
❯ simulation_my_doosan_gazebo_controller
  simulation_inverse_kinematic_pkg
```

Select the ```simulation_my_doosan_gazebo_controller``` launch file and press `Enter` button. Now the simulation is running in the CITROS server, and the results will be automatically uploaded to the CITROS database.

```bash
created new batch_id: <batch_id / batch name>. Running on Citros cluster. See https://citros.io/batch/<batch_id / batch name>.
```

## Results
To get and process the simulation results, execute [built-in Jupiter Notebook](https://citros.io/robotic_arm/blob/citros-ready/notebooks/Robotic%20arm%20analysis%20example.ipynb).
You can see the different behavior of the last joint on these plots:

![png](img/citros3.png "CITROS example")