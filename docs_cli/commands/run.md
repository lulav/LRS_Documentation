## run
The `run` command launches a simulation either locally on your machine, or remotely on the Citros cluster.

### prerequisites:
Ensure that the project has been built and sourced, for example:
    
    $ colcon build
    $ source install/local_setup.bash

If you'd like to run your simulation remotely, you would also need to make sure:
1. You're logged in (via `citros login`).
2. You've built and pushed a docker image of your project (using `citros docker-build-push`).
3. Your `.citros` directory is synched with the remote repository (using `citros commit` and `citros push`). 

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | Specifies the project's working directory. Defaults to `.`|
|`-d`, `--debug` | Sets the logging level to debug.|
|`-v`, `--verbose` | Enables verbose console output.|
|`-s`, `--simulation_name` | Identifies the simulation you want to run. This is the name of the JSON file (excluding the `json` suffix) in the `simulations` folder. If you don't provide a simulation name, an interactive menu will display allowing you to select from the available simulations.|
|`-b`, `--batch_id` | Batch ID. Intended for Citros internal use only - DO NOT USE.|
|`-n`, `--batch_name` | Assigns a descriptive name for this simulation run, e.g. according to its settings and/or parameter setup. You can disable this option requirement via `settings.json`. If disabled, and no name is given, the default name will be the date and time.|
|`-m`, `--batch_message` | Provides a descriptive message for this simulation run, e.g. according to its settings and/or parameter setup. This can also be disabled via `settings.json`.|
|`-i`, `--run_id` | Simulation run ID. Intended for Citros internal use only - DO NOT USE.|
|`-c`, `--completions` | Sets the number of completions (simulation runs). Defaults to 1 if not specified.|
|`-r`, `--remote` | Executes the simulation remotely on the cluster. See prerequisites above for details.|
|`-k`, `--key` | Authentication key. Intended for Citros internal use only - DO NOT USE.|
|`-l`, `--lan_traffic` | A flag which causes the simulation to receive LAN ROS traffic.|
|`--branch` | The git branch name citros should use when running you simulation remotely. Defaults to active branch. For remote run only, will be ignored otherwise.|
|`--commit` | The git commit hash citros should use when running you simulation remotely. defaults to latest commit. For remote run only, will be ignored otherwise.|


If no simulation name was provided, an interactive session will begin, and you will be prompted to select a simulation from the list of available simulations (via up, down and enter keys). 

### example:

    $ citros run
    ? Please choose the simulation you wish to run 
    ❯ simulation_cannon_analytic
      simulation_cannon_numeric

**Note:** the `-n` and `-m` flags are mandatory by default. If you would like them to be optional, you can set the `force_batch_name` and `force_message` flags in `settings.json` to `"False"`. In that case, batch names will default to the date and time the simulation was run. 

