---
sidebar_position: 40
sidebar_label: 'Aerosandbox'
---

# Aerosandbox

# Overview
The project is a simulation tool developed using ROS 2 (Robot Operating System) nodes and leverages the Aerosandbox Python library for aerodynamic calculations. Its primary objective is to simulate gliding flight scenarios for a Cessna 152 aircraft in the event of engine failure.

You can find more information about this useful aerodynamics library on [Aerosandbox official website](https://github.com/peterdsharpe/AeroSandbox). All project installation, code overview and usage details are also available on the project's [GitHub page](https://github.com/citros-garden/aerosandbox_cessna).

![jpg](img/cessna152.jpg "https://en.wikipedia.org/wiki/File:Cessna_152_PR-EJQ_(8476096843).jpg")

# Prerequisites

1. Please make sure you have all the [necessary softwares](https://citros.io/doc/docs_tutorials/getting_started/#softwares-to-work-with-citros) to work with CITROS installed on your computer.
2. Install [Visual Studio code](https://code.visualstudio.com/download).
3. We strongly recommend that you work with [dockers](https://citros.io/doc/docs_tutorials/dockerfile_overview/). However, if you wish to work without dockers, please refer to the .devcontainer [directory](https://github.com/citros-garden/aerosandbox_cessna/tree/main/.devcontainer) in project's repo, the dependencies you need are in the ```Dockerfile``` file.
4. (Optional) Install [FoxGlove](https://docs.foxglove.dev/docs/introduction).

:::note
If you use the provided docker file (or devcontainer) all packages are preinstalled so no action is needed. 
:::


# Table of Contents
- [Installation](#installation)
- [Workspace Overview](#workspace-overview)
- [CITROS Initialization](#citros-initialization)
- [Scenario](#scenario)
- [Running the Scenario Using CITROS](#running-the-scenario-using-citros)
- [Results](#results)

# Installation
```bash
git clone git@github.com:citros-garden/aerosandbox_cessna.git
```

# Workspace Overview

The Aerosandbox simulation has the following ROS parameters:

|Parameter	|Description	|Package
|--|--|--
h_0	|Initial altitude (m)	|aerosandbox_cessna		
v_0	|Initial velocity (knots)	|aerosandbox_cessna	
publish_freq	|Frequency of publishing	|aerosandbox_cessna	

This project contains only one launch file ```aerosandbox_cessna.launch.py```. This file will be used for CITROS launch. 

|Launch File	|Description	|Package
|--|--|--
aerosandbox_cessna.launch.py	|Aerosandbox simulation launch file  |aerosandbox_cessna


# CITROS Initialization

1. [Install CITROS](https://citros.io/doc/docs_tutorials/getting_started/#installation).
2. Follow [these steps](https://citros.io/doc/docs_tutorials/getting_started/#initialization) to Initialize CITROS.

Now you can see .CITROS directory in the explorer.

Check our [Getting Started](https://citros.io/doc/docs_tutorials/getting_started/) guide for additional information.


# Scenario
The user provides flight parameters as input parameters to configure the simulation. These parameters are essential for defining the initial conditions of the simulated flight.

Once the simulation is initiated, the ROS nodes orchestrate the execution. The simulation takes into account various flight dynamics and aerodynamic factors to model the gliding behavior of the Cessna 152. The maximum gliding distance depends on plane's aerodynamic parameters, initial altitude and initial velocity. To find it this code uses iPOPT optimal problem solver under the hood (iPOPT included into AeroSandbox).

The output of the simulation comprises critical flight data, such as altitude, velocity, and other relevant parameters, recorded over time intervals. These results are published via ROS topics, allowing for real-time data visualization, analysis, and integration with other ROS-based systems.

For this example, let's check how far the Cessna can glide with engine failure depending on initial altitude. To find it out, we need to set up parameters and launch CITROS simulation.


After CITROS initialization we can start configuring simulation setup. For remote launch we can set up the maximum performance available: timeout, CPU, GPU and Memory. To perform it, we need to define them in ```.citros/simulations/simulation_aerosandbox_cessna.json```. The recommended setup is minimum 180 seconds timeout, 2 CPU, 2 GPU and 1024 MB of Memory. Don't forget to save the file!

You can find the default parameter setup in ```.citros/parameter_setups/default_param_setup.json```. [CITROS CLI](https://citros.io/doc/docs_cli) provides an opportunity to use basic NumPy functions (such as distributions) and even user-defined functions. In case we want to find out how far the Cessna can glide with engine failure, we need to launch a batch with several simulations and a distribution for one of the ROS parameters (initial altitude, in this case). This parameter will be different for each simulation:

```json
"h_0": {
    "function": "my_func.py:func_with_context",
    "args": [1000.0]
},
```

The ```my_func.py``` file should contain:
```python
def func_with_context(num, citros_context):
    return num + float(citros_context['run_id'])*1000
```

This function will set the ```h_0``` parameter in range from 1000 to 1000+1000*n, where n = number of runs.

# Running the Scenario Using CITROS

<Tabs>

<TabItem value="local" label="Running Locally">

## Running Locally
Since all [the preparations](https://citros.io/doc/docs_tutorials/getting_started/) done, we can launch it locally (your project should be built and sourced before that):
```bash 
>>> citros run -n 'aerosandbox_cessna' -m 'local test run'
? Please choose the simulation you wish to run:
❯ aerosandbox_cessna
```
Select the launch file (should be the only one here) by pressing ```Enter``` button and wait for the output in the terminal. To plot the local run results you can use [FoxGlove](https://docs.foxglove.dev/docs/introduction). Check [this guide](https://citros.io/doc/docs_tutorials/#visualization-with-foxglove) for additional info.

```bash
created new batch_id: <batch_run / batch name>. Running locally.
+ running batch [<batch_run / batch name>], description: local test run, repeating simulations: [1]
+ + running simulation [0]
...
```

![gif](img/gif0.gif "FoxGlove example")

</TabItem>
<TabItem value="cloud" label="Running in Cloud">

## Preparations
To prepare the project for the Cloud Run, follow [our guide](https://citros.io/doc/docs_tutorials/getting_started/#upload-to-citros-server).


## Running 
Finally, we can run it in the cloud! Simply add `-r` to the terminal command: 
```bash 
citros run -n 'aerosandbox_cessna' -m 'cloud test run' -r
? Please choose the simulation you wish to run:
❯ aerosandbox_cessna
```

Select the launch file (should be the only one here) by pressing `Enter` button. Now the simulation is running in the CITROS server, and the results will be automatically uploaded to the CITROS database.

```bash
created new batch_id: <batch_id / batch name>. Running on Citros cluster. See https://citros.io/batch/<batch_id / batch name>.
```

:::tip
The best way to use all the innovative capabilities of CITROS is through it's Web interface. Follow [this manual](https://citros.io/doc/docs/simulations/sim_overview) to easily launch a simulation on CITROS Web platform.
:::

</TabItem>

</Tabs>

# Results

To get and process the simulation results, we can use built-in Jupiter Notebook support. Navigate to our ```Code``` project page, open the Notebooks folder and click on the notebook file. 

![png](img/citros3.png "CITROS example")

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';