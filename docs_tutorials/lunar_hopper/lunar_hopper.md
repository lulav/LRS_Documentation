---
sidebar_position: 60
sidebar_label: 'Lunar Hopper'
---

# Lunar Hopper

## Overview
The Lunar Hopper project is a lunar exploration planning project aimed at solving the intricate problem of lunar hopper missions. It focuses on optimizing the spacecraft's trajectory across the whole flight. Leveraging state-of-the-art optimal problem-solving algorithms, specifically the [MPOPT Python library](https://mpopt.readthedocs.io/en/latest/), this project seeks to determine the most efficient path for maximizing the distance traveled during the mission. It empowers users to customize essential parameters such as spacecraft mass, fuel quantity, thrust, and specific impulse. 

![jpg](img/hopper0.jpg "Did you find an easter egg?)")

## Prerequisites

1. Please make sure you have all the [necessary softwares](https://citros.io/doc/docs_tutorials/getting_started/#softwares-to-work-with-citros) to work with CITROS installed on your computer.
2. Install [Visual Studio code](https://code.visualstudio.com/download).
3. We strongly recommend that you work with [dockers](https://citros.io/doc/docs_tutorials/dockerfile_overview/). However, if you wish to work without dockers, please refer to the .devcontainer [directory](https://github.com/citros-garden/lunar_hopper/tree/main/.devcontainer) in project's repo, the dependencies you need are in the ```Dockerfile``` file.
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
git clone git@github.com:citros-garden/lunar_hopper.git
```

2. Open the repository in the [VScode Dev Container](https://citros.io/doc/docs_tutorials/getting_started/#open-project-in-vscode-dev-container).

## Workspace Overview

The Lunar Hopper simulation has the following ROS 2 parameters:

|Parameter	|Description	|Package
|--|--|--
m_fuel_0	|Initial fuel mass date	|lunar_hopper	
m_fuel_f	|Final fuel mass	|lunar_hopper
dry_mass	|dry mass	|lunar_hopper
F_thrustmax	|The maximum amount of thrust	|lunar_hopper
Isp	|Specific impulse		|lunar_hopper	
publish_freq	|Frequency of publishing	|lunar_hopper

:::note
- Variables with '_0' are initial conditions.
- Variables with '_f' are final conditions.
:::

This project contains only one launch file which will be used for CITROS launch. 

|Launch File	|Description	|Package
|--|--|--
lunar_hopper.launch.py	|Lunar Hopper simulation launch file 	|lunar_hopper


## CITROS Initialization

1. [Install CITROS](https://citros.io/doc/docs_tutorials/getting_started/#installation).
2. Follow [these steps](https://citros.io/doc/docs_tutorials/getting_started/#initialization) to Initialize CITROS.

Now you can see .citros directory in the explorer.

## Scenario
The optimal trajectory computes by solving non-linear OCP (optimal control problems) in the standard Bolza form, by using pseudo-spectral collocation methods and adjusted, using an additional real dynamic function. <br/> 
The OCP solver used in this example is [MPOPT]((https://mpopt.readthedocs.io/en/latest/)) (based on [IPOPT](https://en.wikipedia.org/wiki/IPOPT#:~:text=IPOPT%2C%20short%20for%20%22Interior%20Point,the%20EPL%20(formerly%20CPL).)) library modified by Lulav Space team. <br />
The main goal of the project is to find the optimal way to "hop" on the Moon as far as possible with given vessel parameters.<br/>

For this tutorial, let's check how far the Hopper can hop on the Moon depending on Specific Impulse.
For that, we will launch a simulation batch with a distribution of Specific impulse parameter, starting from 200 and up to 300.
The parameter setup is listed in ```.citros/parameter_setups/default_param_setup.json```. <br/>

```json
{
    "packages": {
        "lunar_hopper": {
            "lunar_hopper": {
                "ros__parameters": {
                    "m_fuel_0": 27.5,
                    "m_fuel_f": 0.0,
                    "dry_mass": 30.0,
                    "Fthrustmax": 208.0,
                    "Isp": {
                        "function": "my_func.py:func_with_context",
                        "args": [200]
                    },
                    "publish_freq": 10.0
                }
            }
        }
    }
}
```

The ```my_func.py``` file should contain:
```python
def func_with_context(num, citros_context):
    return num + float(citros_context['run_id'])*10
```

This function will set the ```Isp``` parameter in range from 200 to 200+10*n, where n = number of runs.

Learn more about parameter setup and defining custom functions in [Directory parameter_setups](https://citros.io/doc/docs_cli/structure/citros_structure/#directory-parameter_setups) and [Adding Functions to Parameter Setup](https://citros.io/doc/docs_cli/configuration/config_params) pages.

In addition to parameter setup, you can configure the simulation performance setup (timeout, CPU, GPU and Memory) as well.
These parameters can be found in ```.citros/simulations/simulation_lunar_hopper.json```. <br/>
Look in [Directory simulations page](https://citros.io/doc/docs_cli/structure/citros_structure#directory-simulations) for more information.

## Running the Scenario Using CITROS

### Running Locally

First, we recommended to update the simulation performance timeout to 600 seconds:

 ```json 
{
    ...
    "parameter_setup": "default_param_setup.json",
    "storage_type": "MCAP",
    "timeout": 600
}
 ```

Then, ensure that the project has been [built and sourced](https://citros.io/doc/docs_tutorials/getting_started/#build-the-project).<br/>

Now we can launch it locally:
```bash 
>>> citros run -n 'Lunar_hopper' -m 'local test run'
? Please choose the simulation you wish to run:
❯ lunar_hopper
```
Select the launch file (should be the only one here) by pressing ```Enter``` button and wait for the output in the terminal. 

```bash
created new batch_id: <batch_run / batch name>. Running locally.
+ running batch [<batch_run / batch name>], description: local test run, repeating simulations: [1]
+ + running simulation [0]
...
```

All the results will be saved under `.citros/runs/[simulation_name]` folder.

To plot the local run results you can use [Foxglove](https://citros.io/doc/docs_tutorials/#visualization-with-foxglove).

### Running in Cloud

First, we recommended to update the simulation performance parameters:
- CPU: 4
- GPU: 4  
- Memory: 4096 MB
- Timeout: 600 seconds


```json
{
    "CPU": 4,
    "GPU": 4,
    "MEM": 4096,
    ...
    "timeout": 600
}
```

Then, [Upload project to CITROS Server](https://citros.io/doc/docs_tutorials/getting_started/#upload-to-citros-server). 

Finally, we can run it in the cloud! Simply add `-r` to the terminal command: 
```bash 
citros run -n 'Lunar_hopper' -m 'cloud test run' -r
? Please choose the simulation you wish to run:
❯ lunar_hopper
```

Select the launch file (should be the only one here) by pressing `Enter` button. Now the simulation is running in the CITROS server, and the results will be automatically uploaded to the CITROS database.

```bash
created new batch_id: <batch_id / batch name>. Running on Citros cluster. See https://citros.io/batch/<batch_id / batch name>.
```

## Results
To get and process the simulation results, execute [built-in Jupiter Notebook](https://citros.io/lunar_hopper/blob/main/notebooks/lunar_hopper_notebook_example.ipynb). 

The graph below shows us the hopping distance depending on Specific impulse:

![png](img/citros3.png "CITROS example")