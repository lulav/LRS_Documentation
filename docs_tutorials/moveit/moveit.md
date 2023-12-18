---
sidebar_position: 90
sidebar_label: 'MoveIt'
---

# MoveIt Example Using CITROS

# Overview
This project is designed to wrap the official MoveIt 2 simulation [example](https://moveit.picknik.ai/main/doc/tutorials/tutorials.html). The node provides ROS parameters that allow users to adjust various MoveIt 2 Motion Planning Framework specifications, enabling the analysis of how these changes affect the Franka Emika Panda robot behavior. 

All project installation, code overview and usage details are also available on the project's [GitHub page](https://github.com/citros-garden/moveit-example).

![png](img/moveit0.png "MoveIT")

# Prerequisites

- [CITROS](https://citros.io/doc/docs_cli/overview/cli_install/)
- [numpy](https://numpy.org/)
- [MoveIt package](https://moveit.ros.org/)

You can use PiP for CITROS and Numpy installation
```bash
python3 -m pip install citros numpy 
```

:::note
If you use the provided docker file (or devcontainer) all packages are preinstalled so no action is needed. 
:::


# Table of Contents
- [Installation](#installation)
- [Workspace Overview](#workspace-overview)
- [CITROS Initialization](#citros-initialization)
- [Scenario](#scenario)
- [Running the scenario using CITROS](#running-the-scenario-using-citros)
- [Results](#results)

# Installation
```bash
git clone git@github.com:citros-garden/moveit-example.git
```

# Workspace Overview

The MoveIt simulation has many ROS parameters files, we need to use only ```moveit_example``` package's parameters:

|Parameter	|Package	|Description
|--|--|--
orientation	|moveit_example	|Arm target orientation
pos_x	|moveit_example	|Arm target position by X axis
pos_y	|moveit_example	|Arm target position by Y axis
pos_z |moveit_example	|Arm target position by Z axis


The official MoveIt tutorial has number of launch files, but for this project we only need to use two launch files:

|Launch File	|Package	|Description
|--|--|--
moveit_example_headless.launch.py	|moveit_example	|MoveIt headless launch file 	
moveit_example.launch.py	|moveit_example	|MoveIt launch file with RViz



# CITROS Initialization

After all the prerequisites are met, we can start configuring our project. The starting point is the MoveIt devcontainer loaded and running, CITROS CLI is installed and ready.

```bash 
>>> citros init
Checking internet connection...
Checking ssh...
Updating Citros...
Waiting for repo to be ready...
Citros repo successfully cloned from remote.
Creating new citros branch `master`.
Creating an initial commit.
Default branch of remote 'origin' set to: master
Citros successfully synched with local project.
You may review your changes via `citros status` and commit them via `citros commit`.
Initialized Citros repository.
```
Now you can see ```.citros``` folder in the explorer.

Check our [Getting Started](https://citros.io/doc/docs/index.md) guide for additional information.

# Scenario
The ROS node interfaces with the official MoveIt simulation example, leveraging the Franka Emika Panda robot model and simulated environment. This integration allows users to observe how the robot behaves in a controlled environment. User can set up the target state of the Arm (rotation and positions by three axes).

TODO!!!

After CITROS initialization we can start configuring simulation setup. For remote launch we can set up the maximum performance available: timeout, CPU, GPU and Memory. To perform it, we need to define them in ```.citros/simulations/simulation_moveit_example.json```. The recommended setup is minimum 120 seconds timeout, 2 CPU, 2 GPU and 2048 MB of Memory. Don't forget to save the file!

You can find the default parameter setup in ```.citros/parameter_setups/default_param_setup.json```. [CITROS CLI](https://citros.io/doc/docs_cli) provides an opportunity to use basic NumPy functions (such as distributions) and even user-defined functions. 

# Running the scenario using CITROS

<Tabs>

<TabItem value="local" label="Running Locally">

## Running Locally
Since all the preparations done, we can launch it locally (your project should be built and sourced before that):
```bash 
>>> citros run -n 'moveit_example' -m 'local test run'
? Please choose the simulation you wish to run:


TODO!!!!!


```
Select the launch file by pressing ```Enter``` button and wait for the output in the terminal. To plot the local run results you can use FoxGlove.

```bash
created new batch_id: <batch_run / batch name>. Running locally.
+ running batch [<batch_run / batch name>], description: local test run, repeating simulations: [1]
+ + running simulation [0]
...
```

![gif](img/foxglove1.gif "FoxGlove example")

</TabItem>
<TabItem value="cloud" label="Running in Cloud">


## Syncing Project's Setup
CITROS account is required for cloud usage. Follow the instructions on [CITROS Website](https://citros.io/auth/login) to register a new one, or check the [CLI documentation](https://citros.io/doc/docs_cli) for logging in. To complete the following steps, it is assumed that the user is registered, logged in and has met all requirements for Web Usage.
Now we can synchronize our project settings with CITROS server:
```bash 
>>> citros commit
>>> citros push
```

## Uploading Docker Image to CITROS Cloud
We need to build and push a Docker container image to the CITROS server:
```bash 
>>> citros docker-build-push
Logging in to docker...
...
```
## Running 
Finally, we can run it in the cloud! Simply add `-r` to the terminal command: 
```bash 
citros run -n 'moveit_example' -m 'cloud test run' -r
? Please choose the simulation you wish to run:



TODO!!!!!



```

Select the launch file by pressing `Enter` button. Now the simulation is running in the CITROS server, and the results will be automatically uploaded to the CITROS database.

```bash
created new batch_id: <batch_id / batch name>. Running on Citros cluster. See https://citros.io/batch/<batch_id / batch name>.
```

:::tip
The best way to use all the innovative capabilities of CITROS is through it's Web interface. Follow [this manual](https://citros.io/doc/docs/simulations/sim_overview) to easily launch a simulation on CITROS Web platform.
:::

</TabItem>

</Tabs>

# Results
CITROS Web provides a powerful data analysis package, which is a comprehensive solution for data query, analysis and visualization. With its extensive features, you can quickly and easily extract valuable insights from your data. To use it, Jupiter Notebook support is built-in. 
Navigate to our ```Code``` project page, open the Notebooks folder and click on the notebook file. Here you can see the usual Jupiter editor interface: you can add blocks of code or built-in Markdown engine, run and save notebook and control the Python kernel.

You can find all the data analysis package guides and API reference [here](https://citros.io/doc/docs_data_analysis).

Let's quickly go through the key points of using a Jupiter Notebook and fetching data from a database.

Run the ```simulation_turtlebot3_sim_cont``` simulation and copy your batch id (we will need it later).

To start with, we need to import all the necessary modules:

```python
import numpy as np
import matplotlib.pyplot as plt
from citros_data_analysis import data_access as da
from prettytable import PrettyTable, ALL
import json
from platform import python_version
```




TODO!!!!


import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';