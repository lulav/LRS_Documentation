---
sidebar_position: 120
sidebar_label: 'MoveIT'
---


# MoveIT

## Overview
This project is designed to wrap the [MoveIT Robotic Arm Simulation project](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) and integrate it with CITROS platform. The node provides ROS 2 parameters that allow users to adjust target arm joints position. In addition, the Inverse Kinematic ROS 2 node was added.

![png](img/arm0.png "Arm")

## Prerequisites

1. Please make sure you have all the [necessary softwares](https://citros.io/doc/docs_tutorials/getting_started/#softwares-to-work-with-citros) to work with CITROS installed on your computer.
2. Install [Visual Studio code](https://code.visualstudio.com/download).
3. We strongly recommend that you work with [dockers](https://citros.io/doc/docs_tutorials/dockerfile_overview/). However, if you wish to work without dockers, please refer to the .devcontainer [directory](https://github.com/citros-garden/moveit/tree/main/.devcontainer) in project's repo, the dependencies you need are in the ```Dockerfile``` file.
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
git clone git@github.com:citros-garden/moveit.git
```
2. Open the repository in the [VScode Dev Container](https://citros.io/doc/docs_tutorials/getting_started/#open-project-in-vscode-dev-container).

## Workspace Overview

The MoveIT simulation has the following ROS 2 parameters:

|Parameter	|Description |Package
|--|--|--
orientation		|Arm orientation 	|moveit_example
pos_x		|Arm position by X axis  	|moveit_example
pos_y		|Arm position by Y axis   |moveit_example
pos_z		|Arm position by Z axis   |moveit_example

This project contains two launch files: 

|Launch File	|Description |Package
|--|--|--
moveit_example.launch.py		|MoveIT launch file with RViz |moveit_example
moveit_example_headless.launch.py	|MoveIT launch file for headless launch |moveit_example


## CITROS Initialization

1. [Install CITROS](https://citros.io/doc/docs_tutorials/getting_started/#installation).
2. Follow [these steps](https://citros.io/doc/docs_tutorials/getting_started/#initialization) to Initialize CITROS.

Now you can see .citros directory in the explorer.

## Scenario
This scenario is used to show the most useful way to interact with robotic arm using inverse kinematic.
User provides target position and target rotation of arm by three axes as input parameters to configure the simulation. The robotic arm will change its position trying to meet the target conditions. 

In this tutorial, let's check how the robotic arm's behavior changes depending on target position. <br />
For that, we will launch a batch simulation with a random distribution ranging between 0.0 and 0.5, for one of the position parameters  (`pos_z`).
The parameter will be set by a NumPy random function and its  setup is listed in ```.citros/parameter_setups/default_param_setup.json```: <br/>

```json
...
        "moveit_example": {
            "moveit_example": {
                "ros__parameters": {
                    "orientation": 1.0,
                    "pos_x": 0.28,
                    "pos_y": -0.2,
                    "pos_z": {
                        "function": "numpy.random.uniform",
                        "args": [0.0, 0.5]
                    }
                }
            }
        },
...
```

Learn more about parameter setup and defining custom functions in [Directory parameter_setups](https://citros.io/doc/docs_cli/structure/citros_structure/#directory-parameter_setups) and [Adding Functions to Parameter Setup](https://citros.io/doc/docs_cli/configuration/config_params) pages.

In addition to parameter setup, you can configure the simulation performance setup (timeout, CPU, GPU and Memory) as well.
These parameters can be found in ```.citros/simulations/simulation_moveit_example_headless.json``` for headless launch and in ```.citros/simulations/simulation_moveit_example.json``` for launch with RViz. <br/>
Look in [Directory simulations page](https://citros.io/doc/docs_cli/structure/citros_structure#directory-simulations) for more information.

## Running the Scenario Using CITROS

### Running Locally

First, we recommend to update the simulation performance timeout to 180 seconds:

 ```json 
{
    ...
    "parameter_setup": "default_param_setup.json",
    "storage_type": "MCAP",
    "timeout": 180
}
 ```

Then, ensure that the project has been [built and sourced](https://citros.io/doc/docs_tutorials/getting_started/#build-the-project).<br/>

Now we can launch it locally:
```bash 
>>> citros run -n 'moveit_example' -m 'local test run'
? Please choose the simulation you wish to run:
❯ simulation_moveit_example_headless
```
Select the ```simulation_moveit_example_headless``` launch file and press ```Enter``` button and wait for the output in the terminal.

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
- Timeout: 180 seconds


```json
{
    "CPU": 4,
    "GPU": 4,
    "MEM": 4096,
    ...
    "timeout": 180
}
```

Then, [Upload project to CITROS Server](https://citros.io/doc/docs_tutorials/getting_started/#upload-to-citros-server). 

Finally, we can run it in the cloud! Simply add `-r` to the terminal command: 
```bash 
citros run -n 'moveit_example' -m 'cloud test run' -r
? Please choose the simulation you wish to run:
❯ simulation_moveit_example_headless
```

Select the ```simulation_moveit_example_headless``` launch file and press `Enter` button. Now the simulation is running in the CITROS server, and the results will be automatically uploaded to the CITROS database.

```bash
created new batch_id: <batch_id / batch name>. Running on Citros cluster. See https://citros.io/batch/<batch_id / batch name>.
```