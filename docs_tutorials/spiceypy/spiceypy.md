---
sidebar_position: 80
sidebar_label: 'SpiceyPy Using CITROS'
---

# SpiceyPy Using CITROS

# Overview
This project is developed using ROS nodes and leverages the SpiceyPy library, a Python implementation of NASA's NAIF Spice toolkit. Its primary purpose is to provide the orbital trajectory information of the Cassini spacecraft relative to Saturn's barycenter within specified time intervals.

You can find more information about SpiceyPy library on [SpiceyPy official website](https://spiceypy.readthedocs.io/en/v2.0.0/index.html). All project installation, code overview and usage details are also available on the project's [GitHub page](https://github.com/citros-garden/spiceypy).

![png](img/Example0.png "Plot")

# Prerequisites

- [CITROS](https://citros.io/doc/docs_cli/overview/cli_install/)
- [numpy](https://numpy.org/)
- [spiceypy](https://spiceypy.readthedocs.io/en/stable/)

```bash
python3 -m pip install citros spiceypy numpy 
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
git clone git@github.com:citros-garden/spiceypy.git
```

# Workspace Overview

The SpiceyPy simulation has the following ROS parameters:

|Parameter	|Package	|Description
|--|--|--
start_t	|spiceypy_cassini	|Initial date	
finish_t	|spiceypy_cassini	|Final date	
publish_freq	|spiceypy_cassini	|Frequency of publishing


This project contains only one launch file ```spiceypy.launch.py```. This file will be used for CITROS launch. 

|Launch File	|Package	|Description
|--|--|--
spiceypy_cassini.launch.py	|spiceypy_cassini	|SpiceyPy Cassini simulation launch file 	


# CITROS Initialization

After all the prerequisites are met, we can start configuring our project. The starting point is the SpiceyPy devcontainer loaded and running, CITROS CLI is installed and ready.

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
This simple scenatio can be used to find the Cassini Spacecraft trajectory relative to Saturn's barycenter within specified time intervals.

Users can input the desired time bounds, and the project utilizes SpiceyPy's powerful capabilities to retrieve accurate and precise orbital data for the Cassini spacecraft during the specified period.

The output of the simulation comprises critical flight data, such as altitude, velocity, and other relevant parameters, recorded over time intervals. These results are published via ROS topics, allowing for real-time data visualization, analysis, and integration with other ROS-based systems.

This example is used to show the CITROS ability to implement any useful library, such as popular SpiceyPy NASA lib for spacecraft tracking. The project's setup is also showing how user can save all the simualtion results in one place for any number of simulations, and share these results with coworkers. 
The other powerfull feature is Jupiter Notebook built-in, and Data analisys package, usage of which will be shown below.

After CITROS initialization we can start configuring simulation setup. For remote launch we can set up the maximum performance available: timeout, CPU, GPU and Memory. To perform it, we need to define them in ```.citros/simulations/simulation_spiceypy_cassini.json```. The recommended setup is minimum 180 seconds timeout, 2 CPU, 2 GPU and 1024 MB of Memory. Don't forget to save the file!

You can find the default parameter setup in ```.citros/parameter_setups/default_param_setup.json```. [CITROS CLI](https://citros.io/doc/docs_cli) provides an opportunity to use basic NumPy functions (such as distributions) and even user-defined functions. 

# Running the scenario using CITROS

<Tabs>

<TabItem value="local" label="Running Locally">

## Running Locally
Since all the preparations done, we can launch it locally (your project should be built and sourced before that):
```bash 
>>> citros run -n 'spiceypy_cassini' -m 'local test run'
? Please choose the simulation you wish to run:
❯ spiceypy_cassini
```
Select the launch file (should be the only one here) by pressing ```Enter``` button and wait for the output in the terminal. To plot the local run results you can use FoxGlove.

```bash
created new batch_id: <batch_run / batch name>. Running locally.
+ running batch [<batch_run / batch name>], description: local test run, repeating simulations: [1]
+ + running simulation [0]
...
```

![png](img/Example1.png "FoxGlove example")

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
citros run -n 'spiceypy_cassini' -m 'cloud test run' -r
? Please choose the simulation you wish to run:
❯ spiceypy_cassini
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

Run the ```spiceypy_cassini``` simulation and copy your batch id (we will need it later).

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
 'size': '1000 kB',
 'sid_count': 1,
 'sid_list': [0],
 'topic_count': 2,
 'topic_list': ['/config', '/spiceypy_cassini/state'],
 'message_count': 4001
}
```
As you can see in the output above, we've got some information about our simulation run (batch): data size, sid information and a list of topics. 

Now we are ready to do some simple research and draw some plots. All MatPlotLib capabilities available here, but the [CITROS Data Analysis](https://citros.io/doc/docs_data_analysis) package provides it's own powerful plotting functions (also based on MatPlotLib):

```python
df = citros.topic('/spiceypy_cassini/state').sid([0]).data(['data.data[0]', 'data.data[1]', 'data.data[2]'])

fig, ax = citros.plot_3dgraph(df, 'data.data[0]', 'data.data[1]', 'data.data[2]', 
                 scale = False, title = 'Results', set_x_label='X', set_y_label='Y', set_z_label='Z',
                 legend = True, )

ax.set_box_aspect(aspect=None, zoom=0.9)
ax.view_init(10, 10, 0)
fig.tight_layout() 
```
You can see the Cassini trajectory around Saturn:
![png](img/citros2.png "CITROS example")


Let's go further:
```python
from datetime import datetime, timedelta

def generate_fixed_length_date_range(start_date, end_date, length):
    delta = (end_date - start_date) / (length - 1)
    date_range = [start_date + i * delta for i in range(length)]
    return date_range

# Define the start and end dates
start_date = datetime(2004, 6, 20)
end_date = datetime(2004, 12, 1)

# Define the desired fixed length
length = len(vel)

# Generate the fixed length date range
date_range = generate_fixed_length_date_range(start_date, end_date, length)

plt.plot(date_range, vel)
```
This graph shows us the Cassini altitude:

![png](img/citros3.png "CITROS example")

All of this data (project setup, simulation runs as well as simulation results and notebook code) is saved into CITROS database and available not only for user, but also for all user's team, who can access these results, check the simulation setups, edit the Jupiter Notebook and export the results. 



import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';