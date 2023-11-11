---
sidebar_position: 20
sidebar_label: 'Aerosandbox'
---

# Aerosandbox

# Overview
The project is a simulation tool developed using ROS (Robot Operating System) nodes and leverages the Aerosandbox Python library for aerodynamic calculations. Its primary objective is to simulate gliding flight scenarios for a Cessna 152 aircraft in the event of engine failure.

You can find more information about this useful aerodynamics library on [Aerosandbox official website](https://github.com/peterdsharpe/AeroSandbox). All project installation, code overview and usage details are also available on the project's [GitHub page](https://github.com/citros-garden/aerosandbox_cessna).

![jpg](img/cessna152.jpg "https://en.wikipedia.org/wiki/File:Cessna_152_PR-EJQ_(8476096843).jpg")

# Prerequisites

- [CITROS](https://citros.io/doc/docs_cli/overview/cli_install/)
- [numpy](https://numpy.org/)
- [aerosandbox[full]](https://pypi.org/project/AeroSandbox/)

```bash
python3 -m pip install citros aerosandbox[full] numpy 
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
git clone git@github.com:citros-garden/aerosandbox_cessna.git
```

# Workspace Overview

The Aerosandbox simulation has the following ROS parameters:

|Parameter	|Package	|Description
|--|--|--
h_0	|aerosandbox_cessna	|Initial altitude (m)	
v_0	|aerosandbox_cessna	|Initial velocity (knots)
publish_freq	|aerosandbox_cessna	|Frequency of publishing


This project contains only one launch file ```aerosandbox_cessna.launch.py```. This file will be used for CITROS launch. 

|Launch File	|Package	|Description
|--|--|--
aerosandbox_cessna.launch.py	|aerosandbox_cessna	|Aerosandbox simulation launch file 


# CITROS Initialization

After all the prerequisites are met, we can start configuring our project. The starting point is the Aerosandbox devcontainer loaded and running, CITROS CLI is installed and ready.

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

# Running the scenario using CITROS

<Tabs>

<TabItem value="local" label="Running Locally">

## Running Locally
Since all the preparations done, we can launch it locally (your project should be built and sourced before that):
```bash 
>>> citros run -n 'aerosandbox_cessna' -m 'local test run'
? Please choose the simulation you wish to run:
❯ aerosandbox_cessna
```
Select the launch file (should be the only one here) by pressing ```Enter``` button and wait for the output in the terminal. To plot the local run results you can use FoxGlove.

```bash
created new batch_id: <batch_run / batch name>. Running locally.
+ running batch [<batch_run / batch name>], description: local test run, repeating simulations: [1]
+ + running simulation [0]
...
```

![gif](img/gif0.gif "FoxGlove example")

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
CITROS Web provides a powerful data analysis package, which is a comprehensive solution for data query, analysis and visualization. With its extensive features, you can quickly and easily extract valuable insights from your data. To use it, Jupiter Notebook support is built-in. 
Navigate to our ```Code``` project page, open the Notebooks folder and click on the notebook file. Here you can see the usual Jupiter editor interface: you can add blocks of code or built-in Markdown engine, run and save notebook and control the Python kernel.

You can find all the data analysis package guides and API reference [here](https://citros.io/doc/docs_data_analysis).

Let's quickly go through the key points of using a Jupiter Notebook and fetching data from a database.

Run the ```aerosandbox_cessna``` simulation and copy your batch id (we will need it later).

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
batch_id = '<batch_run / batch name>'
citros = da.CitrosDB(batch = batch_id)
citros.info().print()
```

The last command returns general batch info:
```python
{
 'size': '261 kB',
 'sid_count': 10,
 'sid_list': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9],
 'topic_count': 2,
 'topic_list': ['/aerosandbox_cessna/state', '/config'],
 'message_count': 1010
}
```
As you can see in the output above, we've got some information about our simulation run (batch): data size, sid information and a list of topics. 

Now we are ready to do some simple research and draw some plots. All MatPlotLib capabilities available here, but the [CITROS Data Analysis](https://citros.io/doc/docs_data_analysis) package provides it's own powerful plotting functions (also based on MatPlotLib):

```python
fig1, ax1 = plt.subplots()
citros.time_plot(ax1, 
                 topic_name = '/aerosandbox_cessna/state', 
                 var_name = 'data.data[0]', 
                 time_step = 1, 
                 sids = [0], 
                 y_label = 'X coords', title_text = 'X coords vs. Time')
```
As you can see, the gliding duration varies for different sids:
![png](img/citros2.png "CITROS example")


Let's go further:
```python
# Defining the list of altitudes

h_0 = [i for i in range(1000,11000, 1000)]


# Setting Dataframe

df = citros.topic('/aerosandbox_cessna/state').set_order({'sid':'asc'}).data('data.data[0]')
sid_list = list(set(df['sid']))
data0_list = []
for s in sid_list:
    id_max = df[df['sid'] == s]['rid'].idxmax()
    data0_list.append(df['data.data[0]'].loc[id_max])

fig, ax = plt.subplots()

#Adjusting colors
c = np.random.choice(50, 10, replace=False)
scatter = ax.scatter(h_0, data0_list,c=c)
6
# Create legend entries for each point
legend_labels = [str(i) for i in range(10)]

# Initialize a list to store legend handles
legend_handles = []

# Loop through the points and create legend entries with matching colors
for i, label in enumerate(legend_labels):
    color = scatter.to_rgba(c[i])  # Get the color of the corresponding point
    legend_handles.append(plt.Line2D([0], [0], marker='o', color='w', label=label, markerfacecolor=color, markersize=10))

# Add the legend with custom handles
legend1 = ax.legend(handles=legend_handles, loc="upper left", title="sid")
ax.add_artist(legend1)
ax.grid()
# plt.scatter(h_0, data0_list, cmap='plasma')
# # plt.plot(h_0, data0_list)
ax.set_ylabel('Gliding distance, m')
ax.set_xlabel('Initial altitude, m')
ax.set_title('Maximum gliding distance vs Initial altitude')
# plt.legend()

```
This graph shows us Maximum gliding distance depending of Initial altitude:  

![png](img/citros3.png "CITROS example")





import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';