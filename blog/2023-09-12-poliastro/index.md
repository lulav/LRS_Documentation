---
slug: Poliastro Example with CITROS
title: Poliastro Example with CITROS
authors: [gtep]
tags: [CITROS]
---

Blog posts support [Docusaurus Markdown features](https://docusaurus.io/docs/markdown-features), such as [MDX](https://mdxjs.com/).

## Example Overview 🌐 

![jpg](img/img0.jpg "https://images.nasa.gov/details/0202375")

This project contains three simple examples using Poliastro lib for Python 3:
1. `Poliastro_simple_orbit`. It returns vessel orbital coordinates around Earth between time bounds from input apoapsis and periapsis altitudes. The result is an ephemerides of orbit (actually a part of it between given time bounds) with zero right ascension of the ascending node, argument of the pericenter and true anomaly for simplicity. 
2. `Poliastro_maneuver`. This package provides three orbits for the Hohmann transition: an initial orbit, an intermediate orbit, and a final orbit. Takes the radius of the initial orbit and the radius of the final orbit as input. You will get the ephemerides of these orbits, not the trajectory! 
3. `Poliastro_atmo_drag`. A simple example showing the effect of aerodynamic drag forces on an artificial satellite on low Earth orbit. Takes Earth diameter, drag coefficient, Keppler orbit parameters and maximum simulation time as inputs. The result is a plot of altitude by time and the flight time before hitting the surface.

You can find all information about used functions and mathematical explanation on the [Poliastro Website](https://docs.poliastro.space/en/stable/). All project installation, code overview and usage details also available in the project [GitHub page](https://github.com/citros-garden/poliastro).

## CITROS Usage 🛸
Although you can get simulation results using FoxGlove, the best way to work with such simulations and process the results is CITROS! With its power, it is possible to create complex data processing scenarios, including the construction of more complex graphs, mathematical analysis and other high-level processing methods.

### CITROS Installation 🛫

First of all, to use all the powerfull CITROS features usage requires CITROS installation: follow the instructions on the CITROS CLI [GitHub page](https://github.com/lulav/citros_cli).

### Configuring the Project ⚙️
After all the prerequisites done, we can start configuring our project. The starting point is the Lunar_Starship devcontainer loaded and running, CITROS CLI is installed and ready.
1. Initialize CITROS:
```bash 
citros init
```
Now you can see ```.citros``` folder in the explorer.

2. Configuring the setup. We need to set up the maximum perfomance available: timeout, CPU, GPU and Memory. To perform it, we need to define it in the ```.citros/simulations/simulation_turtlebot3.json```. The recommended setup is minimum 180 seconds timeout, 2 CPU, and 2048 MB of Memory. Don't forget to save the file!

3. Configuring the params setup. You can find default setup in ```.citros/parameter_setups/default_param_setup.json```. [CITROS CLI](https://github.com/lulav/citros_cli) provides an opportinuty to use basic NumPy functions (such as distributions) and even user-defined functions, but let's keep it default for now. The examples have the following parameters:

    Poliastro Atmospheric Drag simulation:

    $$
    \begin{array}{|c|c|}
    \hline
    \text{Parameter} & \text{Description} \\
    \hline
    earth\_r & \text{Earth radius} \\
    a & \text{Semimajor axis} \\
    ecc & \text{Eccentricity} \\
    inc & \text{Inclination} \\
    raan & \text{Right Ascension of the Ascending Node} \\
    argp & \text{Argument of periapsis} \\
    nu & \text{True anomaly} \\
    c\_d & \text{Drag coefficient} \\
    t\_limit & \text{Maximum simulation duration} \\
    publish~freq & \text{frequency of publishing} \\
    \hline
    \end{array}
    $$

     
    Poliastro Simple Orbit simulation:

    $$
    \begin{array}{|c|c|}
    \hline
    \text{Parameter} & \text{Description} \\
    \hline
    apo\_r & \text{Apoapsis alitude} \\
    peri\_r & \text{Periapsis alitude} \\
    start\_t & \text{Start date and time} \\
    finish\_t & \text{Final date and time} \\
    publish~freq & \text{frequency of publishing} \\
    \hline
    \end{array}
    $$

    Poliastro Maneuver simulation has only 2 ROS parameters: ```r_init``` - initial orbital altitude, and ```r_final``` - final orbital altitude.

:::note
Don't forget to save the file!
:::
4. Launch files. This project contains three launch files:
     * ```poliastro_atmo_drag.launch.py``` - for the Poliastro Atmospheric Drag simulation;
     * ```poliastro_maneuver.launch.py``` - for the Poliastro Simple Orbit simulation;
     * ```poliastro_simple_orbit.launch.py``` - for the Poliastro Maneuver simulation.
     
### Syncing the Project's Setup 📡
Now we can sync our project settings with CITROS server:
```bash 
citros commit
citros push
```

:::tip

CITROS CLI in addition to other advantages also provides automatic ROS bag recording option, which allows user to use saved simulation results and export them! :)

:::

### Running Locally 🛋️
Since all the preparations done, we can launch it locally (your project should be built and sourced before that, check the instructions above):
```bash 
citros run -n 'poliastro' -m 'local test run'
```
Select the launch file (should be the only one here) by pressing ```Enter``` button and wait for the output in the terminal. To plot the local run results you can use FoxGlove.

![gif](img/main.gif "FoxGlove example")
![png](img/maneuver.png "FoxGlove example")

### Uploading Docker Image to CITROS Cloud
We need to build and push Docker container image to the CITROS server:
```bash 
citros docker-build-push
```

### Running in the Cloud 🛰️
Finally, we can run it in the cloud! Simply add ```-r``` to the terminal command: 
```bash 
citros run -n 'poliastro' -m 'cloud test run' -r
```
Select the launch file by pressing ```Enter``` button. Now the simulation is running in the CITROS server, and it will upload results to the CITROS database automaticly.

### CITROS Web Usage
#### Launching project via CITROS Web
The best way to use all the innovative capabilities of CITROS is through it's Web interface. The following manual explains how to run this project in the cloud and how to process the simualtion results.
The starting point is CITROS main page, user is logged in and the project Docker image is built and pushed to the cloud (see the [manual](#uploading-docker-image-to-citros-cloud) above).
1. Go to the ```Repositories``` page clicking on the tab on the top;
2. Find your project and open it;
3. Navigate to the ```Runs``` tab;
4. Click on the ```Run Simulation``` button on the right;
5. Now you can choose the project and the simulation setup from the droplists, set the number of repeats and how many simulations should run in parallel, type the Name of the run and the additional message. This window also shows the perfomance preset.
6. We are ready to go! Start the Batch with the button below.

The simualtion launched! Open the Run you just started in the list on ```Runs``` page to check how is it going. In this page you can find all the runs of this batch. The number of runs here equals to the number of runs you've set before.
Navigate to the Run by clicking on it in the table:
* The main part of this page is a simulation's log. Here you can find all the logging information from all levels: from your code logs up to the CITROS system information.
* The right part of the page provides additional information about Events: the main stages of the simulation run.

![png](img/citros0.png "CITROS example")

#### Working with Integrated Jupiter Notebooks and Data Analysis 🌌
CITROS Web provides powerfull data analisys package, which is comprehensive solution for data query, analysis and visualization. With its extensive features, you can quickly and easily extract valuable insights from your data. To use it, the Jupiter Notebook support is built-in. 
Navigate to our project ```Code``` page, open the Notebooks folder and click on the notebook file. Here you can see the usual Jupiter editor's interface: you can add blocks of code or built-in Markdown engine, run and save notebook and control the Python kernel.

You can find all the data analisys package guides and API reference [here](https://citros.io/doc/docs_data_analysis).

![png](img/citros1.png "CITROS example")

## Extras
### FoxGlove examples
![png](img/atmo_drag0.png "FoxGlove example")

