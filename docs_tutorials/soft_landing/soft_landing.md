---
sidebar_position: 80
sidebar_label: 'Soft Landing'
---

# Soft Landing Tutorial

This is a ROS 2 simulation of soft landing of an object.  
In the ROS 2 system we have two nodes: the first represents the `dynamics` and the second one is the `controller`.
- **System dynamics** - The system's equation of motion is the kinematic equation of a free body fall.  
for more information see [Soft Landing](https://github.com/CITROS-garden/soft_landing)  

- **The controller** - The controller is based on this paper:
*S. Gutman, "Rendezvous and Soft Landing in Closed Form via LQ Optimization," 2019 27th Mediterranean Conference on Control and Automation (MED), Akko, Israel, 2019, pp. 536-540, doi: 10.1109/MED.2019.8798572.*

![jpg](img/soft_landing_control.jpg "soft landing")

## Table of Contents

3. [Installation](#installation)
    1. [Prerequisites](#prerequisites)
    2. [Install & Build](#install--build)
4. [CITROS Integration](#citros-integration)
    1. [Configuring The Project](#configuring-the-project)
5. [Run the Example](#run-the-example)
    1. [The Scenario](#the-scenario)
    2. [Run a Test Simulation Locally with CITROS](#run-a-test-simulation-locally-with-citros)
    3. [Run a Test Simulation In The Web with CITROS](#run-a-test-simulation-in-the-web-with-citros)

## Installation

### Prerequisites
 - [Python 3.8+](https://www.python.org/downloads/)  
 - [VSCode](https://code.visualstudio.com/download) 
 - [Docker](https://www.docker.com/get-started/)  
 
   
### Install & Build 
1. Clone the repository:
   ```sh
    git clone git@github.com:citros-garden/soft_landing.git
   ```

2. open the repository in the VScode:
	```sh
	cd ~/soft_landing
	code .
	```
3. open the repository in the container from VScode with `reopen in container` option.


## CITROS Integration
To use all the powerfull CITROS features usage requires CITROS installation:  
(from the instructions on the CITROS CLI [GitHub page](https://github.com/lulav/CITROS_cli))


**First,reopen the folder localy** then follow the instructions:
```
pip install citros
```  


then login:
```
citros login 
```

enter your email and password, you suppose to see:    

![Alt text](img/image-5.png)

then:

```
 citros init
```
![Alt text](img/image-4.png)



and finally:

```
citros setup-ssh
citros add-remote
```
then check that we all set with the cli command: `CITROS status`

### Configuring The Project
After all the prerequisites done, we can start configuring our project. The starting point is the soft_landing devcontainer loaded and running, CITROS CLI is installed and ready.
1. Initialize CITROS:
```bash 
citros init
```
Now you can see ```.CITROS``` folder in the explorer and at the terminal you can see that:  
![Alt text](img/image-2.png)  

4. Reopen in container
5. source and build:
	```sh
	colcon build
	source install/local_setup.bash
	```

## Run the Example

### The Scenario
Run the example of an object with the initial condition that suppose to land on an ending point.  
The parameters are:  



|     Variable     | Description | package |
| -------- |    ------- |  ------- | 
| r_x0 , r_y0, r_z0 | initial position| dynamics |
| v_x0 ,v_y0,v_z0 | initial velocity |  dynamics |
| g_x , g_y , g_z | gravity vector | dynamics |
| dt | time interval | dynamics |
|  setpoint_r_x , setpoint_r_y , setpoint_r_z , |  ending position |controller |  
|  setpoint_v_x , setpoint_v_y , setpoint_v_z , |  ending velocity |controller |
|  g  | gravity parameter | controller |
| um |  acceleration limit | controller |
| e | stoping condition value | controller |
| dt | time interval | controller |

**All of the above are ROS 2 parameters that could easly change by the user as wish.**  


After completing the CITROS interagition setup we can cheack CITROS by running a test simulation.  

First, set up the parameter of the simulation in the file   `default_param_setup.json` in `.CITROS/parameter_setups` folder.  
Don't forget to save the file!  


``` 
you can read more about changing parameters in  `parameter setups` section in `CITROS_cli` readme.
```  


### Run a Test Simulation Locally with CITROS
#### Syncing Project's Setup
Now we can sync our project settings with CITROS server:
```bash 
citros commit
citros push
```
when everything is setup you can do a test run buy the following command:  
```
citros run -n 'test' -m 'testytest'
```
Then you will ask to choose the launch file you want to run.  
There are two option:

![Alt text](img/image-3.png)

The `simulation_dynamics_controller` launch the dynamics with the controller and `simulation_dynamics` launch only the dynamics.  
Select the launch file by pressing ```Enter``` button and wait for the output in the terminal.  
If the simulation ran perfectly you can run the simlulation in the cloud.

### Run a Test Simulation in The Web with CITROS
Befor uploading the simulation to the cloud check that the parameter file, `default_param_setup.json`  in `.CITROS/parameter_setups` folder is set as you wish and saved.  
That you have build and sourced the project.  
And you have sync the project settings with CITROS server (CITROS commit , CITROS push).  

Then,run the following:
1. We need to build and push Docker container image to the CITROS server:
```bash 
citros docker-build-push
```
 Finally, we can run it in the cloud! Simply add ```-r``` to the terminal command: 
```bash 
citros run -n 'test' -m 'testytest' -r
```
Select the launch file you want by pressing ```Enter``` button. Now the simulation is running in the CITROS server, and it will upload results to the CITROS database automaticly.

Now we all set to run simulations from the web itself.  
At the web,go to the soft landing repo and then to the `Runs` tab.  

start a new simulation by clicking the ![Alt text](img/image-1.png) button.  
then a window will pop:

![Alt text](img/image-6.png)  

As you can see I chose to fill the soft landing repository, the main branch, and run the simulations with the controller.  
Then I chose to repeat 100 times such that CITROS run 10 simulations in parallel.  
Then i clicked on *Run Simulation*.  
The results were:  
![Alt text](img/image-7.png)

And by getting the miss distance and miss velociy i could show a figure of all the runs.  
![Alt text](img/image-8.png)  
The full report with the data acces and error anlyisis you can watch it [here](https://CITROS.io/soft_landing/blob/main/notebooks/Soft_landing_analysis.ipynb).





