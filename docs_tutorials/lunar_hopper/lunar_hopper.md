---
sidebar_position: 60
sidebar_label: 'Lunar Hopper'
---

# Lunar Hopper Optimal Control Example Using CITROS

The Lunar Hopper project is a lunar exploration planning project aimed at solving the intricate problem of lunar hopper missions. It focuses on optimizing the spacecraft's trajectory across the whole flight. Leveraging state-of-the-art optimal problem-solving algorithms, specifically the MPOPT Python library, this project seeks to determine the most efficient path for maximizing the distance traveled during the mission. It empowers users to customize essential parameters such as spacecraft mass, fuel quantity, thrust, and specific impulse. 

![jpg](img/hopper0.jpg "Did you find an easter egg?)")

## Table of Contents
1. [CITROS Usage](#citros-usage)
    1. [CITROS Installation](#citros-installation)
    2. [Configuring the project](#configuring-the-project)
    3. [Running locally](#running-locally)
    4. [Synchronizing the project's setup](#synchronizing-the-projects-setup)
    5. [Uploading Docker image to CITROS cloud](#uploading-docker-image-to-citros-cloud)
    6. [Running in the cloud](#running-in-the-cloud)
    7. [CITROS Web usage and data analysis](#citros-web-usage-and-data-analysis)

## CITROS Usage
Although you can get simulation results using FoxGlove, the best way to work with such simulations and process the results is CITROS! With its power, it is possible to create complex data processing scenarios, including the construction of more complex graphs, mathematical analysis and other high-level processing methods.

### CITROS installation

First of all, to use all the powerfull CITROS features the CLI installation is required: follow the instructions on the CITROS CLI [documentation page](https://citros.io/doc/docs_cli).

### Configuring the project
After all the prerequisites are met, we can start configuring our project. The starting point is the Lunar_hopper devcontainer loaded and running, CITROS CLI is installed and ready.
1. Initialize CITROS:
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
Intialized Citros repository.
```
Now you can see ```.citros``` folder in the explorer.

2. Configuring the setup. We need to set up the maximum performance available: timeout, CPU, GPU and Memory. To perform it, we need to define them in ```.citros/simulations/simulation_lunar_hopper.json```. The recommended setup is minimum 600 seconds timeout, 4 CPU, 4 GPU and 4096 MB of Memory. Don't forget to save the file!

3. Configuring the params setup. You can find the default setup in ```.citros/parameter_setups/default_param_setup.json```. [CITROS CLI](https://citros.io/doc/docs_cli) provides an opportunity to use basic NumPy functions (such as distributions) and even user-defined functions, but let's keep it default for now. Variables with '_0' are initial conditions, and variables with '_f' are final conditions respectively. Don't forget to save the file!

    |Parameter	|Package	|Description
    |--|--|--
    m_fuel_0	|lunar_hopper	|Initial fuel mass date	
    m_fuel_f	|lunar_hopper	|Final fuel mass
    dry_mass	|lunar_hopper	|dry mass
    F_thrustmax	|lunar_hopper	|The maximum amount of thrust
    Isp	|lunar_hopper	|Specific impulse	
    publish_freq	|lunar_hopper	|Frequency of publishing


4. Launch files. This project contains only one launch file ```lunar_hopper.launch.py```. This file will be used for CITROS launch. 

    |Launch File	|Package	|Description
    |--|--|--
    lunar_hopper.launch.py	|lunar_hopper	|Lunar Hopper simulation launch file 	



:::tip

CITROS CLI, in addition to other benefits, also provides an automatic ROS bag recording option, which allows user to use saved simulation results and export them! :)
:::
### Running locally
Since all the preparations done, we can launch it locally (your project should be built and sourced before that):
```bash 
>>> citros run -n 'Lunar_hopper' -m 'local test run'
? Please choose the simulation you wish to run:
❯ lunar_hopper
```
Select the launch file (should be the only one here) by pressing ```Enter``` button and wait for the output in the terminal. To plot the local run results you can use FoxGlove.

```bash
created new batch_id: <your-batch-id-here>. Running locally.
+ running batch [<your-batch-id-here>], description: local test run, repeating simulations: [1]
+ + running simulation [0]
...
```

### Synchronizing the Project's Setup 
CITROS account is required for cloud usage. Follow the instructions on [CITROS Website](https://citros.io/auth/login) to register a new one, or check the [CLI documentation](https://citros.io/doc/docs_cli) for logging in. To complete the following steps, it is assumed that the user is registered, logged in and has met all requirements for Web Usage.
Now we can synchronize our project settings with CITROS server:
```bash 
>>> citros commit
>>> citros push
```

### Uploading Docker image to CITROS cloud
We need to build and push a Docker container image to the CITROS server:
```bash 
>>> citros docker-build-push
Logging in to docker...
...
```

### Running in the cloud
Finally, we can run it in the cloud! Simply add ```-r``` to the terminal command: 
```bash 
>>> citros run -n 'Lunar_hopper' -m 'local test run' -r
? Please choose the simulation you wish to run:
❯ lunar_hopper
```

Select the launch file (should be the only one here) by pressing ```Enter``` button. Now the simulation is running in the CITROS server,  and the results will be automatically uploaded to the CITROS database.

```bash
created new batch_id: <your-batch-id-here>. Running on Citros cluster. See https://citros.io/batch/<your-batch-id-here>.
```

### CITROS web usage and data analysis
#### Launching project via CITROS web
The best way to use all the innovative capabilities of CITROS is through it's Web interface. Follow [this manual](https://citros.io/doc/docs_citros_web/simulations/sim_overview) to easily launch a simulation on CITROS Web platform.

#### Working with integrated Jupiter notebooks and data analysis
CITROS Web provides a powerfull data analysis package, which is a comprehensive solution for data query, analysis and visualization. With its extensive features, you can quickly and easily extract valuable insights from your data. To use it, Jupiter Notebook support is built-in. 
Navigate to our ```Code``` project page, open the Notebooks folder and click on the notebook file. Here you can see the usual Jupiter editor interface: you can add blocks of code or built-in Markdown engine, run and save notebook and control the Python kernel.

You can find all the data analisys package guides and API reference [here](https://citros.io/doc/docs_data_analysis).

Let's quickly go through the key points of using a Jupiter Notebook and fetching data from a database. But to try some brief examples of data analysis using the built-in package, we need to launch a batch with several simulations and a distribution for one of the ROS parameters (Specific impulse, in our case). This parameter will be different for each simulation:

```json
"Isp": {
    "function": "my_func.py:func_with_context",
    "args": [200]
},
```

All necessary things are already configured (we used a user-defined function, you can read more about its usage in the [CITROS CLI](https://citros.io/doc/docs_cli) manual), so you can start the simulation from [CLI](#citros-usage-🛸) with the ```-c 10``` flag: 

```bash 
>>> citros run -n 'Lunar_hopper' -m 'local test run' -r -c 10
? Please choose the simulation you wish to run:
❯ lunar_hopper
```

Or from [Web](#running-in-the-cloud-🛰️):

![png](img/web0.png "CITROS example")

Run the ```Lunar_hopper``` simulation and copy your batch id (we will need it later).

Let's return to our Notebook and check the code: to start with, we need to import all the necessary modules:

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
 'size': '5197 kB',
 'sid_count': 10,
 'sid_list': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9],
 'topic_count': 4,
 'topic_list': ['/config', '/lunar_hopper/control', '/lunar_hopper/state', '/lunar_hopper/time'],
 'message_count': 21931
}
```
As you can see in the output above, we've got some information about our simulation run (batch): data size, sid information and a list of topics. 

Now we are ready to do some simple research and draw some plots. All MatPlotLib capabilities available here, but the [CITROS Data Analisys](https://citros.io/doc/docs_data_analysis) package provides it's own powerful plotting functions (also based on MatPlotLib):

```python
fig1, ax1 = plt.subplots()

citros.time_plot(ax1, 
                 topic_name = '/lunar_hopper/state', 
                 var_name = 'data.data[0]', 
                 time_step = 1, 
                 sids = [0,1,2,3,4,5,6,7,8,9],
                 y_label = 'X coords', title_text = 'X coords vs. Time')
```
As you can see, the maximum altitude varies for different sids:
![png](img/citros2.png "CITROS example")


Let's go further:
```python
# Defining the list of isp

isp_list = [i for i in range(200,300,10)]

# Setting Dataframe

df = citros.topic('/lunar_hopper/state').set_order({'sid':'asc'}).data('data.data[1]')
sid_list = list(set(df['sid']))
data0_list = []
for s in sid_list:
    id_max = df[df['sid'] == s]['rid'].idxmax()
    data0_list.append(df['data.data[1]'].loc[id_max])

print(data0_list)
fig, ax = plt.subplots()

#Adjusting colors
c = np.random.choice(50, 10, replace=False)
scatter = ax.scatter(isp_list, data0_list,c=c)

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
ax.set_ylabel('hopping distance, m')
ax.set_xlabel('Initial altitude, m')
ax.set_title('Maximum hopping distance vs Initial altitude')    
```
This graph shows us the hopping distance depending of Specific impulse:

![png](img/citros3.png "CITROS example")
