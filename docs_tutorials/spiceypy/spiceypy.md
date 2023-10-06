---
sidebar_position: 10
sidebar_label: 'SpiceyPy'
---

## SpiceyPy Using CITROS

This project is developed using ROS nodes and leverages the SpiceyPy library, a Python implementation of NASA's NAIF Spice toolkit. Its primary purpose is to provide the orbital trajectory information of the Cassini spacecraft relative to Saturn's barycenter within specified time intervals.

Users can input the desired time bounds, and the project utilizes SpiceyPy's powerful capabilities to retrieve accurate and precise orbital data for the Cassini spacecraft during the specified period.

You can find more information about SpiceyPy library on [SpiceyPy official website](https://spiceypy.readthedocs.io/en/v2.0.0/index.html). All project installation, code overview and usage details are also available on the project's [GitHub page](https://github.com/citros-garden/spiceypy).

![png](img/Example0.png "Plot")

## Table of Contents

1. [CITROS Usage](#citros-usage)
    1. [CITROS installation](#citros-installation)
    2. [Configuring the project](#configuring-the-project)
    3. [Syncing project's setup](#syncing-projects-setup)
    4. [Running locally](#running-locally)
    5. [Uploading Docker image to the CITROS database and running in the cloud](#uploading-docker-image-to-the-citros-database-and-running-in-the-cloud)
    6. [Running in the cloud](#running-in-the-cloud)
    7. [CITROS Web usage](#citros-web-usage)
2. [Extras](#extras)
    1. [Foxglove examples](#foxglove-examples)

## CITROS Usage
Although you can get simulation results using FoxGlove, the best way to work with such simulations and process the results is CITROS! With its power, it is possible to create complex data processing scenarios, including the construction of more complex graphs, mathematical analysis and other high-level processing methods.

### CITROS installation

First of all, to use all the powerfull CITROS features usage requires CITROS installation: follow the instructions on the CITROS CLI [GitHub page](https://github.com/lulav/citros_cli).

### Configuring the project
After all the prerequisites are met, we can start configuring our project. The starting point is the Poliastro devcontainer loaded and running, CITROS CLI is installed and ready.
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

2. Configuring the setup. We need to set up the maximum performance available: timeout, CPU, GPU and Memory. To perform it, we need to define them in ```.citros/simulations/simulation_spiceypy_cassini.json```. The recommended setup is minimum 180 seconds timeout, 2 CPU, 2 GPU and 1024 MB of Memory. Don't forget to save the file!

3. Configuring the params setup. You can find the default setup in ```.citros/parameter_setups/default_param_setup.json```. [CITROS CLI](https://github.com/lulav/citros_cli) provides an opportunity to use basic NumPy functions (such as distributions) and even user-defined functions, but let's keep it default for now. The SpiceyPy simulation has the following ROS parameters:

    |Parameter	|Package	|Description
    |--|--|--
    start_t	|spiceypy_cassini	|Initial date	
    finish_t	|spiceypy_cassini	|Final date	
    publish_freq	|spiceypy_cassini	|Frequency of publishing

:::note
The date format should be 'MMM DD, YYYY'.
:::

Don't forget to save the file!

4. Launch files. This project contains only one launch file ```spiceypy.launch.py```. This file will be used for CITROS launch. 

    |Launch File	|Package	|Description
    |--|--|--
    spiceypy_cassini.launch.py	|spiceypy_cassini	|SpiceyPy Cassini simulation launch file 	

### Syncing project's setup
Now we can synchronize our project settings with CITROS server:
```bash 
>>> citros commit
>>> citros push
```

:::tip

CITROS CLI, in addition to other benefits, also provides an automatic ROS bag recording option, which allows user to use saved simulation results and export them! :)

:::

### Running locally
Since all the preparations done, we can launch it locally (your project should be built and sourced before that):

```bash 
>>> citros run -n 'spiceypy_cassini' -m 'local test run'
? Please choose the simulation you wish to run:
❯ spiceypy_cassini
```

Select the launch file (should be the only one here) by pressing ```Enter``` button and wait for the output in the terminal. To plot the local run results you can use FoxGlove.

```bash
created new batch_id: <your-batch-id-here>. Running locally.
+ running batch [<your-batch-id-here>], description: local test run, repeating simulations: [1]
+ + running simulation [0]
...
```

![png](img/Example1.png "FoxGlove example")

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
>>> citros run -n 'spiceypy_cassini' -m 'local test run' -r
? Please choose the simulation you wish to run:
❯ spiceypy_cassini
```

Select the launch file (should be the only one here) by pressing ```Enter``` button. Now the simulation is running in the CITROS server, and the results will be automatically uploaded to the CITROS database.

```bash
created new batch_id: <your-batch-id-here>. Running on Citros cluster. See https://citros.io/batch/<your-batch-id-here>.
```

### CITROS web usage
#### Launching project via CITROS web
The best way to use all the innovative capabilities of CITROS is through it's Web interface. The following manual explains how to run this project in the cloud and how to process the simualtion results.
The starting point is CITROS main page, user is logged in and the project Docker image is built and pushed to the cloud (see the [manual](#uploading-docker-image-to-citros-cloud) above).
1. Go to the ```Repositories``` page clicking on the tab on the top;
2. Find your project and open it;
3. Navigate to the ```Runs``` tab;
4. Click on the ```Run Simulation``` button on the right;
5. Now you can select the project and the simulation setup from the drop-down lists, set the number of repeats and how many simulations should run in parallel, type the Name of the run and the additional message. This window also shows the perfomance preset.
6. We are ready to go! Start the Batch with the button below.

The simualtion launched! Open the Run you just started in the list on ```Runs``` page to check how it is going. On this page you can find all the runs of this batch. The number of runs here equals to the number of runs you've set before.
Navigate to the Run by clicking on it in the table:
* The main part of this page is a simulation's log. Here you can find all the logging information from all levels: from your code logs up to the CITROS system information.
* The right part of the page provides additional information about Events: the main stages of the simulation run.


#### Working with integrated Jupiter notebooks and data analysis
CITROS Web provides a powerfull data analysis package, which is a comprehensive solution for data query, analysis and visualization. With its extensive features, you can quickly and easily extract valuable insights from your data. To use it, Jupiter Notebook support is built-in. 
Navigate to our ```Code``` project page, open the Notebooks folder and click on the notebook file. Here you can see the usual Jupiter editor interface: you can add blocks of code or built-in Markdown engine, run and save notebook and control the Python kernel.

You can find all the data analisys package guides and API reference [here](https://citros.io/doc/docs_data_analysis).

Let's quickly go through the key points of using a Jupiter Notebook and fetching data from a database. All necessary things are already configured, so you can start the simulation from [CLI](#citros-usage-🛸) with the ```-c 10``` flag: 

```
>>> citros run -n 'spiceypy_cassini' -m 'local test run' -r
? Please choose the simulation you wish to run:
❯ spiceypy_cassini
```

Or from [Web](#running-in-the-cloud-🛰️):

![png](img/web0.png "CITROS example")

Run the ```spiceypy_cassini``` simulation and copy your batch id (we will need it later).

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

The last command returns general database info:
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

Now we are ready to do some simple research and draw some plots. All MatPlotLib capabilities available here, but the [CITROS Data Analisys](https://citros.io/doc/docs_data_analysis) package provides it's own powerful plotting functions (also based on MatPlotLib):

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

## Extras
### Foxglove examples
<!-- ![gif](img/gif0.gif "FoxGlove example") -->


