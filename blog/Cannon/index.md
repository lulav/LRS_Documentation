# Welcome to Citros CLI

This tutorial will guide you through the Citros CLI interface, using a simple ROS 2 example project to demonstrate the usage, while providing usefull recommendations and best practices. While this is not a comprehensive guide to all Citros CLI commands, it should get you up and running using your own projects with Citros in no time. For further details and an exhaustive guide to the Citros CLI, refer to the [CLI Documentation](https://github.com/lulav/citros_doc/blob/main/docs_cli/index.md).

# Table of contents

1. [The Cannon example project](#the-cannon-example-project)
2. [Working with Citros CLI offline](#cannon-via-citros-offline)
3. [Working with Citros CLI online](#cannon-via-citros-online)


# The Cannon example project

This project is a ROS implementation of the [cannonball simulation](https://nasa.github.io/trick/tutorial/ATutASimpleSim) provided by NASA Johnson Space Center as part of the tutorial for the 
[Trick Simulation Environment](https://nasa.github.io/trick/).

It determines the trajectory and time of impact of a cannon ball that is fired with an initial speed and initial angle, assuming a constant acceleration of gravity (g), and no aerodynamic forces.

![Cannonball](img/CannonInit.png "Cannonball")

Two versions of the simulation are provided: an analytic solution and a numeric integration solution.

## Prerequisites

- [Visual Studio code](https://code.visualstudio.com/download)
- [Docker](https://www.docker.com/)
- [Foxglove](https://foxglove.dev/) (optional)

## Installation
```bash
    $ git clone git@github.com:citros-garden/cannon.git
    $ cd ~/cannon
    $ code .
```
and open the repository inside a container using VScode's *reopen in container* option.

## Build 
```bash
    $ colcon build
    $ source install/local_setup.bash
```

## Run the analytic solution
```bash
    $ ros2 launch scheduler cannon_analytic.launch.py
```

## Run the numeric integration solution
```bash
    $ ros2 launch scheduler cannon_numeric.launch.py
```


Running either of the two simulations will result in the logger output being written to the console.

## Implementation Overview
The project is made out of three ROS nodes - `cannon_analytic`, `cannon_numeric` and `scheduler`. The scheduler node is responsible for driving the simulation by publishing a `scheduler` topic at a given rate (say, 100Hz). The cannon nodes subscribe to this topic, and upon receiving it perform a single calculation step. The rate (`dt`) is a ROS parameter for the scheduler node, which means you may change its value in the `config/params.yaml` file, without the need to recompile. The two cannon nodes also have `params.yaml` files of their own, in which you can set the initial speed and angle, and also the time/integration delta (`dt`).

Additionally, the `scheduler` node subscribes to a `debug` topic, which, together with the provided Foxglove layout, facilitates a play/pause/step/resume functionality. 

## Foxglove
To view a graphical representation of the simulation, you can open [Foxglove](https://foxglove.dev/) and load the `CITROS_Cannon.json` layout file, or create your own layout.

It is recommended to start the simulation in a paused state, and then, once your foxglove layout is ready, resume it via the Play/Pause button. 

To do that, in the `__init__` member function of the `scheduler` node (in `scheduler.py`), change the line

        self.debug_mode = False

to

        self.debug_mode = True
You will need to build (and source) again.

Output example:
![Foxglove screenshot](img/foxglove_screenshot.png)



# Cannon via Citros (offline)

Working with the Citros CLI offline is pretty straight forward, since there are only two thinks you need to do - initialze your Citros repository, and run your project. Additionaly, you may configure your Citros repository to fit your simulation needs, but if all you want to do is run your project via Citros with the default configuration, than only two commands are necessary.

## Prerequisites
But first, let's make sure all the prerequisites for running Citros have been met:
- Open the project (in this case Cannon) inside a VS Code dev-container.
- Build and source your project by running:
    ```bash
        $ colcon build
        $ source install/local_setup.bash
    ```
- in the dev-container's terminal, run
    ```bash
        $ pip install citros
    ```

    You can verify that the installation succeeded by running 
    ```bash
        $ citros -V
        1.2.28
    ```

    to get the Citros CLI version installed.

## Initialization
Alrightythan! you're now ready to run the first command - `init`, which will initialize your local Citros repository:
```bash
$ citros init
User is not logged in. Initialzing Citros locally.
Creating new citros branch `main`.
Creating an initial commit.
Checking out new citros branch `main`.
Intialized Citros repository.
```

This command creates a folder named `.citros` under your project directory, and initializes it as a git repository. The current git branch of the `.citros` repository should always be named as the current branch in your ROS project (i.e. `main` in the example above). In case they are different, Citros will create and/or checkout the branch for you, as can be seen in the output above.

The `.citros` directory contains several files and folders that capture the state of your project and allow you to configure your simulations according to your needs. We will discuss some of them briefly later on. For a full and detailed description of the contents of the `.citros`  directory, refer to the [CLI Documentation](https://github.com/lulav/citros_doc/blob/main/docs_cli/index.md).

## Running a simulation

After your `.citros` repository has been initialized, you're ready to run a Citros simulation, albeit with all the default configurations, by using the `run` command:

```bash
$ citros run -n "my_first_batch" -m "my first Citros simulation!"
? Please choose the simulation you wish to run: 
❯ simulation_cannon_analytic
  simulation_cannon_numeric
```

To fully understand what's going on, we need to familiarize ourselves with three concepts that are core to the way Citros works:
- **simulation** 

    Defined by a ROS 2 launch file. You may have as many launch files as you want in your project, as long as there is at least one. Each launch simulation will correspond to a launch file in your project. When you run a Citros simulation, if you don't specify the name of the simulation (using the `-s` flag), a command-line menue will be presented, in which you can use the up and down arrows to choose the simulation you want. The simulation names will be of the form `simulation_<name of launch_file>`. In the case of the Cannon project, we have two launch files - `cannon_analytic.launch.py` and `cannon_numeric.launch.py`, and as you can see in the output above, we are prompted to choose between them. 

    Each simulation also corresponds to a json file of the same name, which resides under the `.citros/simulations` directory. You may use this file to configure the way your simulation runs. 

    When you run a Citros simulation, a directory for that simulation is created under the `.citros/runs` directory. This directory will contain subdirectories corresponding to **batch**es, a new one created every time you run a simulation. 
- **batch** 

    Defined as a group of one or more simulation runs. Since you can specify one or more simulations runs ('*complitions*') when running a Citros simulation, a **batch** is simply a convinient way to group them together. For instance, in the case of the above example, if we choose `simulation_cannon_analytic` from the menue, the following folder structure will be created: `.citros/runs/simulation_cannon_analytic/my_first_batch/0`. The last folder - `0`, is the folder corresponding to the only run for this batch - when you don't specify the number of completions (i.e. runs) using the `-c` flag, it will default to 1, and the name of each run is a zero based index, incremented by one for each additonal run.
- **run**

    Defined as a single execution of a simulation as defined by the chosen launch file. Launching Citros simulations with multiple runs ('*complitions*') is particulary advantageous when working online, in which case a large number of simulation runs can be simultaneously executed on the Citros cloud.

    The folder corresponding to a simulation run will contain all the information relevant for that run. Look through such a folder after running a simulation and see for yourself. For further details refer to the [CLI Documentation](https://github.com/lulav/citros_doc/blob/main/docs_cli/index.md)


By default, when using the `run` command, you must provide a batch name (using the `-n` flag) and a message (using the `-m` flag). The name you provide will be used as the name of the directory in which all runs for this batch will be saved. If a batch by that name already exists - no worries, Citros will simply add an underscore and an index to the name you provided, thereby keeping the batch directory names unique for each simulation. 

Now that you understand what's going on, choose on of the simulations presented in the menue, press enter and see it run...

## Configuring a simulation

We just ran a simulation a single time with all the default configurations, which is admittedly not that exciting. Let's see how we can turn things up a notch by setting up dynamic parameter evaluation for our simulation, thereby allowing each run within the same batch to have different parameter values.

The `.citros/parameter_setups` directory stores your JSON-formatted parameter setup files. When you initialize your citros repository, a `default_param_setup.json` file is automatically generated. This file consolidates all the default parameters for every node across all the packages in your ROS project, providing a consolidated and easily accessible record of these parameters.

The structured format of the parameter setup files streamlines both the understanding and alteration of parameters for each node in your ROS project. This becomes especially valuable when you're keen to explore the influence of different parameter values on your ROS project's behavior.

In the Cannon project, we have a total of three nodes. Let's look at the parameters for the `analytic_dynamics` node in the `cannon_analytic` package, as defined in the `default_param_setup.json` file.
We can see we have 3 parameters to play around with - `init_speed`, `init_angle` and `dt`. 

Let's say we want to find out the optimal initial angle for the cannon, which will provide the maximum range. Assuming we're completely blancking out on high-school physics, let's randomize the value for this parameter, execute several simulation runs, and see where we get the maximum range. To achieve this, we can simply replace the hard-coded default value with a **function object**. Function objects are json objects comprised of two fields - `function` and `args`. They come in two flavors - numpy and user-defined. For our purposes we can use numpy's random module to generate a normal distribution around a given value:

    "init_angle": {
                    "function": "numpy.random.normal",
                    "args": [45, 15]
                },

This will cause a normal distribution with a standard deviation of 15 around 45 to be evaluated for every simulation run.

Now, if we run
```batch
citros run -n "test_params" -m "testing random initial angle" -c 10
```

and choose `simulation_cannon_analytic` from the menue, the simulation will run 10 times (sequentially), and each time the cannon will have a different initial angle. By looking at the results, we can hopefully come to the conclusion that 45 degrees is the optimal angle. 

For user defined functions and further details about configuring Citros simulations in general, refer to the [CLI Documentation](https://github.com/lulav/citros_doc/blob/main/docs_cli/index.md). 


# Cannon via Citros (online)

