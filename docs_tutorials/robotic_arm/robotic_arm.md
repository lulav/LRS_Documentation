---
sidebar_position: 40
sidebar_label: 'Robotic Arm'
---


# Robotic Arm

# Overview
This project is designed to wrap the [Robotic Arm Simulation project](https://github.com/dvalenciar/robotic_arm_environment) and integrate it with CITROS platform. The node provides ROS parameters that allow users to adjust target arm joints position. In addition, the Inverse Kinematic ROS node was added.

![png](img/arm0.png "Arm")

# Prerequisites

- [CITROS](https://citros.io/doc/docs_cli/overview/cli_install/)
- [numpy](https://numpy.org/)
- [lark](https://github.com/lark-parser/lark)
- [empy](https://pypi.org/project/empy/)
- [catkin_pkg](https://pypi.org/project/catkin-pkg/)
- [libfranka](https://frankaemika.github.io/docs/libfranka.html)
- [ikpy](https://github.com/Phylliade/ikpy)

```bash
python3 -m pip install citros numpy lark empy catkin_pkg ikpy
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
git clone git@github.com:citros-garden/robotic_arm.git
```

# Workspace Overview

The Robotic Arm simulation has the following ROS parameters:

For Forward Kinematic:

|Parameter	|Package	|Description
|--|--|--
j0	|my_doosan_pkg	|First joint target position 	
j1	|my_doosan_pkg	|Second joint target position  	
j2	|my_doosan_pkg	|Third joint target position  
j3	|my_doosan_pkg	|Fourth joint target position  
j4	|my_doosan_pkg	|Fifth joint target position  
j5	|my_doosan_pkg	|Sixth joint target position  

For Inverse Kinematic:

|Parameter	|Package	|Description
|--|--|--
pos0	|inverse_kinematic_pkg	|Arm target position by first axis	
pos1	|inverse_kinematic_pkg	|Arm target position by second axis
pos2	|inverse_kinematic_pkg	|Arm target position by third axis
ori0	|inverse_kinematic_pkg	|Arm target orientation by first axis
ori1	|inverse_kinematic_pkg	|Arm target orientation by second axis 
ori3	|inverse_kinematic_pkg	|Arm target orientation by third axis 


This project contains two launch files: ```simulation_my_doosan_gazebo_controller``` for Forward Kinematic or ```simulation_inverse_kinematic_pkg``` for Inverse Kinematic. These files will be used for CITROS launch. 

|Launch File	|Package	|Description
|--|--|--
my_doosan_gazebo_controller.launch.py	|my_doosan_pkg	|Gazebo Robotic Arm launch file for Forward Kinematic
simulation_inverse_kinematic_pkg.launch.py	|simulation_inverse_kinematic_pkg	|Gazebo Robotic Arm launch file for Inverse Kinematic


# CITROS Initialization

After all the prerequisites are met, we can start configuring our project. The starting point is the Robotic Arm devcontainer loaded and running, CITROS CLI is installed and ready.

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
## Forward kinematics
In this example user provides joints target positions as input parameters to configure the simulation. The robotic arm will change its position trying to meet the target conditions.

## Inverse kinematics
This example uses the Inverse Kinematic calculations to show more useful way to interract with robotic arm. User provides target position and target rotation of arm by three axes as input parameters to configure the simulation. The robotic arm will change its position trying to meet the target conditions.

The output of the simulation comprises all robot's configuration data and other relevant parameters, recorded over time intervals. These results are published via ROS topics, allowing for real-time data visualization, analysis, and integration with other ROS-based systems.

For this example, let's check how the robotic arm's behavior changes depending on target position. To find it out, we need to set up parameters and launch CITROS simulation.


After CITROS initialization we can start configuring simulation setup. For remote launch we can set up the maximum performance available: timeout, CPU, GPU and Memory. To perform it, we need to define them in ```.citros/simulations/simulation_my_doosan_gazebo_controller.json``` (for Forward Kinematic) or ```.citros/simulations/simulation_inverse_kinematic_pkg.json``` (for Inverse Kinematic). The recommended setup is minimum 600 seconds timeout, 4 CPU, 4 GPU and 4096 MB of Memory. Don't forget to save the file!

You can find the default parameter setup in ```.citros/parameter_setups/default_param_setup.json```. [CITROS CLI](https://citros.io/doc/docs_cli) provides an opportunity to use basic NumPy functions (such as distributions) and even user-defined functions. In case we want to find out how the robotic arm's behavior changes, we need to launch a batch with several simulations and a distribution for one of the ROS parameters (last joint position, in this case). This parameter will be different for each simulation:

```json
"j5": {
    "function": "numpy.random.random",
    "args": [0.0, 0.5]
}
```

This function will set the ```j5``` parameter in random range from 0.0 to 0.5.

# Running the scenario using CITROS

<Tabs>

<TabItem value="local" label="Running Locally">

## Running Locally
Since all the preparations done, we can launch it locally (your project should be built and sourced before that):
```bash 
>>> citros run -n 'robotic_arm' -m 'local test run'
? Please choose the simulation you wish to run:
❯ simulation_my_doosan_gazebo_controller
  simulation_inverse_kinematic_pkg
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
citros run -n 'robotic_arm' -m 'cloud test run' -r
? Please choose the simulation you wish to run:
❯ simulation_my_doosan_gazebo_controller
  simulation_inverse_kinematic_pkg
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

Run the ```simulation_my_doosan_gazebo_controller``` simulation and copy your batch id (we will need it later).

To start with, we need to import all the necessary modules:

```python
import numpy as np
import matplotlib.pyplot as plt
from citros_data_analysis import data_access as da
from prettytable import PrettyTable, ALL
import json
from platform import python_version
```

Now we can connect to the simulation database:
```python
batch_id = '<your-batch-id-here>'
citros = da.CitrosDB(batch = batch_id)
citros.info().print()
```

The last command returns general batch info:
```python
{
 'size': '15 MB',
 'sid_count': 10,
 'sid_list': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9],
 'topic_count': 6,
 'topic_list': ['/config', '/joint_states', '/joint_trajectory_controller/state', '/robot_description', '/tf', '/tf_static'],
 'message_count': 16286
}
```
As you can see in the output above, we've got some information about our simulation run (batch): data size, sid information and a list of topics. 

Now we are ready to do some simple research and draw some plots. All MatPlotLib capabilities available here, but the [CITROS Data Analysis](https://citros.io/doc/docs_data_analysis) package provides it's own powerful plotting functions (also based on MatPlotLib):

```python
df=citros.topic('/joint_trajectory_controller/state').sid(0).data(["data.actual.positions"])
list0=[df['data.actual.positions'].loc[n] for n in range(len(df))]

labels = ['Joint 0', 'Joint 1','Joint 2','Joint 3','Joint 4','Joint 5']
plt.plot(list0, label=labels)
plt.xlabel("Time")
plt.ylabel("Joint Position")
plt.title("Joint rotation VS Time")
plt.legend()
```
As you can see, the traveled trajectory varies for different sids:
![png](img/citros2.png "CITROS example")


Let's go further:
```python
# Creating a figure and a grid of subplots
fig, ax = plt.subplots(nrows=4, ncols=3, figsize=(15, 10))  # Adjust grid size and figure size as needed
fig.suptitle("Joint rotation VS Time for different sIds", fontsize=16)

# Flattening the ax array to loop through it
ax = ax.flatten()

# Plotting in loop
for i in range(10):

    # Getting data from dataframe for each sId
    df=citros.topic('/joint_trajectory_controller/state').sid(i).data(["data.actual.positions"])
    list0=[df['data.actual.positions'].loc[n] for n in range(len(df))]

    # Defining lists separately to highlight the changing value
    list04 = [sublist[0:5] for sublist in list0]
    list5 =[sublist[5] for sublist in list0]

    # Plotting with different linestyles
    ax[i].plot(list04, label=labels[0:5])
    ax[i].plot(list5, label=labels[5], linestyle='--')

    ax[i].set_xlabel("Time")
    ax[i].set_ylabel("Joint Position")
    ax[i].set_title("sId=" + str(i))
    ax[i].legend(loc = 'upper left')

# Adjusting layout to prevent overlap
plt.tight_layout()

# Displaying the plot
plt.show()
```
You can see the different behavior of the last joint on these plots:

![png](img/citros3.png "CITROS example")





import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';