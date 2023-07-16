---
sidebar_position: 1
sidebar_label: 'CLI'
hide_title: true
---

```python
# ==============================================
#   ██████╗██╗████████╗██████╗  ██████╗ ███████╗
#  ██╔════╝██║╚══██╔══╝██╔══██╗██╔═══██╗██╔════╝
#  ██║     ██║   ██║   ██████╔╝██║   ██║███████╗
#  ██║     ██║   ██║   ██╔══██╗██║   ██║╚════██║
#  ╚██████╗██║   ██║   ██║  ██║╚██████╔╝███████║
#   ╚═════╝╚═╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝                                        
# ==============================================
```
# CITROS CLI

[![Publish CITROS_CLI to PyPI / GitHub](https://github.com/lulav/citros_cli/actions/workflows/release_to_pypi.yaml/badge.svg)](https://github.com/lulav/citros_cli/actions/workflows/release_to_pypi.yaml)

# Description

This repository is a python package that implements the CITROS CLI API.
It is run by the user inside the ROS project folder. 

# Prerequisites

- [vscode](https://code.visualstudio.com/download)
- [Docker](https://www.docker.com/)
- [Python3](https://www.python.org/downloads/)

# Installation (without code)

        pip instal citros 

# Installation (with code)

1. clone the repo:
    
        git clone git@github.com:lulav/citros_cli.git

2. Within the cloned `citros_cli` folder, open VSCode:
    
        code .

    and reopen the folder inside a Dev Container.

3.  Install the package from the current directory to a global bin folder 

        python3 -m pip install .

    If you are developing the citros_cli package itself, than you can install the package  
    with soft links to dev environment. 

        python3 -m pip install -e .
    
4. Env

| ENV | Description | used in |
| --- | --- | --- |
| `CITROS_DOMAIN` | the main domain, default is `citros.io` | all packages |
| `CITROS_DATA_HOST` | host of the playground PGDB, default comes from simulation job env (citros_worker) and is: `shared-playground-postgresql.ns-citros-shared`. used for uploading BAG. | citros_bag |
| `CITROS_DATA_PORT` | the port of PGDB, default `5432` | citros_bag |
| `CITROS_DATA_DATABASE` | the database to access inside PGDB. default comes from simulation job env (citros_worker) and is: `domain_prefix`. used for uploading BAG. | citros_bag |
| `CITROS_DATA_USERNAME` | the username to access PGDB, default `citros_anonymous`. the username is the username from citros.  | citros_bag |
| `CITROS_DATA_PASSWORD` | the password to access PGDB, default `citros_anonymous`. the password is the id of the user in `citros.user` table.  | citros_bag |

# CLI API commands:

## init
Initialize a citros project. If the user is not logged in, the project will be initialized locally. If the user is logged in, the project will be cloned from the citros remote reopsitory (if this is a new project, an empty project will be cloned).

The initialization includes the creation of the `.citros` directory under your project directory, and the creation of several files and folders therein, which allow you to run a simulation of your project with default configurations and settings. You may configure your project for you specific needs by manually editing these files (see `Project Configuration` below).

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | The working directory of the project. Defaults to `.`|
|`-d`, `--debug` | Set logging level to debug|
|`-v`, `verbose` | Use verbose console prints|
|`-project_name` <proj_name> | Optional name for the project. Defaults to the last folder in the path of *dir*|

example:

    ros@shalev-Inspiron-15-5510:/workspaces/cannon$ citros init
    Intialized Citros project.

Note: you can only run this command (with any effect) once per project. If you try to initialize an existing citros project, you will be notified and no action will be taken. Example:

    ros@shalev-Inspiron-15-5510:/workspaces/cannon$ citros init
    The directory /workspaces/cannon has already been initialized as a citros projct.
    No remotes found. Working locally.

in order to re-initialize an existing citros project, you would first need to delete the existing `.citros` directory for you project.

## setup-ssh
Setup ssh keys for secure communication with the remote citros repository.

### parameters:
parameter|description
|--|--|
|`-d`, `--debug` | Set logging level to debug|
|`-v`, `verbose` | Use verbose console prints|

## status
Get the citros repository status.

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | The working directory of the project. Defaults to `.`|
|`-d`, `--debug` | Set logging level to debug|
|`-v`, `verbose` | Use verbose console prints|

## add-remote
Add remote citros repo to existing local repo.

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | The working directory of the project. Defaults to `.`|
|`-d`, `--debug` | Set logging level to debug|
|`-v`, `verbose` | Use verbose console prints|

## commit
Commit changes to the local citros repository.

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | The working directory of the project. Defaults to `.`|
|`-d`, `--debug` | Set logging level to debug|
|`-v`, `verbose` | Use verbose console prints|
|`-m`, `--message` | Commit message|

## push
push commits to the remote citros repository.

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | The working directory of the project. Defaults to `.`|
|`-d`, `--debug` | Set logging level to debug|
|`-v`, `verbose` | Use verbose console prints|
|`-b`, `--branch` | The branch name. Defaults to `master` if not given.|

## login
Login to CITROS.

### parameters:
parameter|description
|--|--|
|`-d`, `--debug` | Set logging level to debug|
|`-v`, `verbose` | Use verbose console prints|
|`-username` | The user's username (email).|
|`-password` | The user's password|

After entering the command, if either the username or password was not given, you will be prompted for your email (the username) and password.

example:

    ros@shalev-Inspiron-15-5510:/workspaces/cannon$ citros login
    email: shalev@lulav.space
    Password: 
    User logged in.


## logout
Logout of CITROS.

### parameters:
parameter|description
|--|--|
|`-d`, `--debug` | Set logging level to debug|
|`-v`, `verbose` | Use verbose console prints|

example:

    ros@shalev-Inspiron-15-5510:/workspaces/cannon$ citros logout
    User logged out.
## sync
Sync the ros project to citros.

This command will parse your ROS2 project, extract the names of all the packages, 
nodes, parameters and launch files (and some other metadata), and create two files 
under the `.citros` directory:
  - `project.json`
  - `parameter_setups/default_param_setup.json`

Do not change these files, as they will be overwritten on the next sync. In order to create another (non default) parameter setup, simply copy and paste the `default_param_setup.json` file under a differnt name (within the `parameter_setups` folder).

Use the `-v` option for a verbose output. example:
    
    ros@shalev-Inspiron-15-5510:/workspaces/mass_spring_dumper$ citros sync -v
    --- using self.CITROS_DOMAIN = http://citros.local
    parsing package: /workspaces/cannon/src/scheduler
        node: scheduler
    parsing package: /workspaces/cannon/src/cannon_numeric
        node: numeric_dynamics
    parsing package: /workspaces/cannon/src/cannon_analytic
        node: analytic_dynamics


### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | The working directory of the project. Defaults to `.`|
|`-d`, `--debug` | Set logging level to debug|
|`-v`, `verbose` | Use verbose console prints|
|`-n, -name` <proj_name> | The name of the project. Defaults to the last folder in the path of *dir*|
|`-c`, `--commit` | Automatically commit after sync is done.|

## list
List all simulation names.

### parameters:
parameter|description
|--|--|
|`-d`, `--debug` | Set logging level to debug|
|`-v`, `verbose` | Use verbose console prints|

examle:

    ros@shalev-Inspiron-15-5510:/workspaces/cannon$ citros list
    1. simulation_cannon_analytic
    2. simulation_cannon_numeric


## run
Run a simulation - either locally on your computer, or remotely on the citros cluster.

### prerequisites:
The project has been built and sourced, e.g.:
    
    colcon build
    source install/local_setup.bash

If you'd like to run your simulation remotely, you would also need to make sure:
1. You're logged in (via `citros login`).
2. You've built and pushed a docker image of your project (using `citros docker-build-push`).
3. Your `.citros` directory is synched with the remote repository (using `citros commit` and `citros push`). 

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | The working directory of the project. Defaults to `.`|
|`-d`, `--debug` | Set logging level to debug|
|`-v`, `verbose` | Use verbose console prints|
|`-s`, `--simulation_name` <sim_name> | The simulation name, i.e. the name of the json file (*without* the `json` suffix) under the `simulations` folder, of the simulation you'd like to run. If a simulation name is not given, than an interactive menue will come up and you'll be able to choose a simulation from all available ones.|
|`-b`, `--batch_id` | Batch id. For citros internal use only - DO NOT USE.|
|`-n`, `--batch_name` | Batch name. Give a descriptive name for this simulation run, e.g. according to its settings and/or parameter setup. You may turn off the mandating of this option via `settings.json`, in which cae the default name will be the date and time.|
|`-m`, `--batch_message` | Batch message. Give a descriptive message for this simulation run, e.g. according to its settings and/or parameter setup. You may turn off the mandating of this option via `settings.json`|
|`-i`, `--run_id` | Simulation run id. For citros internal use only - DO NOT USE.|
|`-c`, `--completions` | Number of completions (simulation runs). Defaults to 1 if not given.|
|`-t`, `--timeout` | Simulation timeout in seconds. After this time has elapsed since the start of a simulation run, it will be terminated. Defaults to 60 seconds if not given.|
|`-r`, `--remote` | Run the simulation remotely on the cluster. See `prerequisites` above for details.|


## upload_bag
experimental.

upload bag to citros.

### parameters:
parameter|description
|--|--|
|`-d`, `--debug` | Set logging level to debug|
|`-v`, `verbose` | Use verbose console prints|
|`-bag` | The name of the baf file to upload|
|`-batch_run_id` | The batch run id of the run during which the bag was recorded.|
|`-sid` | The simulation run id of the run during which the bag was recorded.|



## docker-login
Login to docker-hub.

### parameters:
parameter|description
|--|--|
|`-d`, `--debug` | Set logging level to debug|
|`-v`, `verbose` | Use verbose console prints|


## docker-build-push
Builds and pushes the project to docker-hub.

### prerequisites:
you must be logged in to docker-hub to run this command.

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | The working directory of the project. Defaults to `.`|
|`-d`, `--debug` | Set logging level to debug|
|`-v`, `verbose` | Use verbose console prints|
|`-n`, `--image_name` | The requested image name (e.g. the project name). Defaults to the last folder in the path of dir |

---

# Project Configuration

## Adding distributions to parameter setup
In order to define a distribution in your parameter setup, simply replace any constant parameter value with a `distribution object`, which is demontrated in the following example: 

    {
        "distribution_type": "POISSON",
        "distribution_param1": 1.0,
        "param1_type": "FLOAT",
        "distribution_param2": 1,
        "param2_type": "INT"
    }

where `distribution_type` is one of the following:

    ["NORMAL", "EXPONITIONAL", "LAPLACE", "POISSON", "POWER", "UNIFORM", "ZIPF", "VONMISES", "RAYLEIGH", "FLOAT", "STRING"]

and `param1_type` and `param2_type` are either `"INT"` or `"FLOAT"`.

If your distribution only requires one parameter, the second will be ignored, but all 5 key/value pairs are required.

Here is an example of how you could modify your param_setup.json file:

    {
    "packages": [
        {
            "JokeTeller_Bot": [
                {
                    "HumorEngine": {
                        "ros__parameters": {
                            "joke_intensity": 11,
                            "joke_distribution": {
                                "distribution_type": "NORMAL",
                                "distribution_param1": 1.0,
                                "param1_type": "FLOAT",
                                "distribution_param2": 1,
                                "param2_type": "INT"
                            },
                            "funny_numbers": [
                                42,
                                69,
                                {
                                    "distribution_type": "POISSON",
                                    "distribution_param1": 1.0,
                                    "param1_type": "FLOAT",
                                    "distribution_param2": 1,
                                    "param2_type": "INT"
                                },
                                17
                            ]
                        }
                    }
                }
            ]
        }
    ]
}
