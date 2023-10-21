# Citros Repository File Structure

The following folder and file structure is automatically generated for you (when you run `citros init`):

```
└── .citros
    ├── notebooks
    ├── parameter_setups
        └── functions
            └── routes.js
        └── default_param_setup.json
    ├── reports
    ├── runs
    ├── simulations
        ├── simulation_foo.json
        └── simulation_bar.json
    ├── workflows
        └──default_flow.json
    ├── citros_repo_id
    ├── project.json
    ├── settings.json
    └── user_commit

```

## Directory `notebooks`
<details>
  <summary>Description</summary>
  TODO
</details>

## Directory `parameter_setups`
<details>

<summary>Description</summary>

The `parameter_setups` directory stores your JSON-formatted parameter setup files. When you initialize your citros repository, a `default_param_setup.json` file is automatically generated. This file consolidates all the default parameters for every node across all the packages in your ROS project, providing a consolidated and easily accessible record of these parameters.

The file `default_param_setup.json` will not be overwritten during citros `init`, `run` or `status` commands. Nevertheless, it is recommended to duplicate this file under a different name within the `parameter_setups` directory before making any modifications. This practice ensures your custom setups are preserved and allows you to experiment with various parameter configurations.
    
The structured format of the parameter setup files streamlines both the understanding and alteration of parameters for each node in your ROS project. This becomes especially valuable when you're keen to explore the influence of different parameter values on your ROS project's behavior. Take, for instance, a static parameter value like 42. Instead of hard-coding it, you could use a *function object* to derive a value from a normal distribution centered at 42. The introduction of function objects broadens your horizons, enabling you to use any numpy function or even craft user-defined functions for meticulous computational adjustments. A prime example is when parameter values are intricate, making them cumbersome to hard-code; in such scenarios, you can devise a function to fetch them from a file. In essence, this newfound flexibility paves the way for limitless computational and manipulative possibilities for your parameters.
    
To learn how to add functions to parameter setups, please refer to the [Adding functions to parameter setup](/docs_cli/configuration/config_params.md) section.

</details>

## Directory `reports`
<details>
  <summary>Description</summary>
  TODO
</details>

## Directory `runs`
<details>
<summary>Description</summary>
The runs directory stores data and metadata about each run of your simulations. Its structure is as follows:

```
└── Simulation Name
    └── Batch Name
        └── Run ID
            ├── bag
                ├── simulation_foo.json
                └── simulation_bar.json
            ├── config
            ├── msgs
            ├── citros.log
            ├── environment.json
            ├── info.json
            └── metrics.csv
```
- Simulation Name: These directories are named after each of the simulations defined in the simulation files. For every simulation file that is run, a corresponding directory is created here. Each Simulation Name directory may include multiple Batch Name directories.

- Batch Name: This directory holds a batch of simulation runs. A batch consists of multiple runs of the same simulation with different parameters.

- Run ID: Each unique simulation run has its own directory, identified by a Run ID. Under this directory, there are several files and sub-directories:

- `bag`: This sub-directory holds the recorded data from the simulation run. It includes:

- bag_0.db3: This is a ROS bag file that contains all the messages that were sent during the simulation. The default bag format is `sqlite3` (hence the db3 postfix), but you may also use the `mcap` format. See [simulations](#directory-simulations).

- metadata.yaml: A file holding metadata information associated with the bag file.

- `config`: This sub-directory contains YAML files (pkg1.yaml, pkg2.yaml, etc.) for each package in your ROS project, detailing the actual parameters used in the simulation. If you used any functions in your parameter setup, the values appearing here will be those that were evaluated according to the function you defined.

- `msgs`: This sub-directory contains all the ROS msg files you may have in your project, each under yet another sub-directory with a name corresponding to the package the msg file belongs to.
            
- `citros.log`: A standard log file that was active during the simulation run, documenting actions and events throughout the simulation.
            
- `environment.json`: A file capturing a snapshot of your environment variables and Python packages at the time of the simulation run.

- `info.json`: A JSON file containing general metadata about the run, such as batch ID, batch name, datetime of the run, user's Git commit and branch information, and Citros' Git commit and branch information, as well as a hash of the bag file.

- `metrics.csv`: A CSV file recording system performance metrics during the simulation run, including CPU usage, total memory, available memory, used memory, and memory usage percentage.

These files collectively provide a comprehensive record of each simulation run, the conditions under which it was run, and the results it produced. This makes it easy to reproduce and understand the results of each simulation.

</details>

## Directory `simulations`
<details>
<summary>Description</summary>

The `simulations` directory stores your JSON-formatted simulation files.

A simulation json file is an auto-generated file corresponding to each launch file in your ROS project. For instance, a launch file named `foo.launch.py` will have a corresponding `simulation_foo.json` file. This file outlines the details necessary to run the corresponding simulation, specifying parameters, resources, and launch files.

Here's a breakdown of its typical structure and content:

- `description`: This is a descriptive field for the simulation setup. You can modify it to better describe your specific simulation.

- `parameter_setup`: This field points to the parameter setup JSON file that will be used for this simulation. By default, it points to `default_param_setup.json`, but you can point it to any custom parameter setup file you created in the `parameter_setups` directory.

- `launch_file`: Specifies the ROS launch file that will be used to start the simulation. For instance, `foo.launch.py`.

- `timeout`: This is the maximum time (in seconds) the simulation is allowed to run. The default is 60 seconds. If the simulation does not conclude within this timeframe, it will be terminated.

- `GPU`: Specifies the number of GPU resources required for the simulation. The default is 0, indicating that no GPU resources are needed.

- `CPU`: Specifies the number of CPU resources required for the simulation. The default is 2.

- `MEM`: Specifies the amount of memory required for the simulation in megabytes, e.g., 265.

- `storage_type`: This setting determines the storage format for the ROS bag files generated during the simulation's runs. The possible valid value are `SQLITE3` (default) and `MCAP`.

You can modify these fields to suit your simulation needs, just remember to save your customized version under a different name to prevent overwriting during citros `init`, `run`, or `status` commands.

</details>

## Directory `workflows`
<details>
<summary>Description</summary>

The `workflows` directory stores your JSON-formatted workflow files.

A flow.json file (e.g. `default_flow.json` which is auto-generated during `citros init`) is a user-crafted file used to automate and manage the flow of simulations in a citros repository. This file controls when the flow is triggered, which simulations are running, the post-processing analysis using Jupyter notebooks, and the recipients of the final reports. Here is a breakdown of its structure and content:

- `trigger`: This field specifies the event that initiates the flow. It is usually tied to some form of version control event, like a Git push, but can be configured according to the user's needs.

- `simulations`: This is an array of simulations to be run, specified as pairs of simulation name and the number of times to run them. For example, ["sim1", 17] means the simulation "sim1" will be run 17 times. Multiple simulations can be listed and each will be run the specified number of times.

- `notebooks`: This is a list of Jupyter notebooks used for post-processing analysis of the simulation results. For example, ["nb1.ipynb", "nb2.ipynb"] means these two notebooks will be run once the simulations complete, with the results used as their input data.

- `recipients`: This is a list of email addresses that will receive the reports generated from the notebooks' analysis.

The flow.json file helps to streamline and automate your citros repository by tying together simulation runs, data analysis, and report distribution into a single manageable file. You can customize it to suit the specifics of your project.

</details>

## File `project.json`
<details>
<summary>Description</summary>
The project.json file is a key component of your Citros repository. It contains metadata about your ROS project, and is automatically generated by the citros `init`, `run` and `status` commands. Here's a description of its top-level fields:

- `citros_cli_version`: The Citros CLI version installed.

- `cover`: A placeholder for a potential image that represents the project.

- `description`: A string for providing a detailed description of the project.

- `git`: The git repository URL associated with the project.

- `image`: A name that corresponds to the docker image of the project.

- `is_active`: A boolean flag indicating whether the project is active or not.

- `launches`: An array for storing metadata about launch files associated with the project. 
  
    **Note**: these are the global launch files, which are not associated with any specific package. Generally, they are less commonly used. For package launch files, see inside the list of [*packages*](#packages-array).

- `license`: A string indicating the license of the project.

- `name`: The name of the project. *Note*: this is the only field that you may edit and it will not be overwritten during subsequent citros commands.

- [`packages`](#packages-array): An array of objects that describe the ROS packages that exist within the project.

- `path`: The directory path to the project.

- `readme`: The contents of the project's README file.

- `tags`: An array of strings for tagging and categorizing the project.

#### `packages` Array

In the `packages` array, each object describes a specific package within the project. These objects contain similar information to the top-level fields, with additional fields:

- `maintainer`: The maintainer of the package.

- `maintainer_email`: The email address of the maintainer.

- [`nodes`](#nodes-array): An array of objects describing each node in the package, including their parameters and entry points.

- `package_xml`: The path to the package's XML file.

- `setup_py`: The path to the package's `setup.py` file. For python ROS projects only.

- `cmake`: The path to the package's `CMakeLists.txt` file. For C++ ROS projects only.

- `parameters`: An array of objects that describe the package-level parameters, i.e. parameters which are not associated with any node. As with node-level parameters, this includes their name, type, and value.

#### `nodes` Array

The `nodes` array contains objects that describe the ROS nodes within a package. Each object includes the following fields:

- `entry_point`: The entry point for the node, typically the function that should be executed when the node is run.

- `name`: The name of the node.

- `parameters`: An array of objects that describe the parameters associated with the node, including their name, type, and value.

- `path`: The path to the node's Python file.

</details>

## File `settings.json`
<details>
<summary>Description</summary>

The settings.json file holds configuration settings for your Citros repository. Here is a breakdown of each field in 
this file:

- `name`: The name of the current settings profile. This can be useful if you want to maintain different sets of settings for different contexts (e.g., 'default_settings', 'debug_settings', etc.).

- `force_message`: This is a boolean setting (in string format). If set to "True", it enforces that a descriptive message is provided for each batch of simulation runs. This can be helpful for keeping track of the purpose or characteristics of each run batch.

- `force_batch_name`: Similar to force_message, this is a boolean setting (in string format). If set to "True", it enforces that a unique name is provided for each batch of simulation runs. This can be useful for organizing and identifying different batches of runs.

</details>
