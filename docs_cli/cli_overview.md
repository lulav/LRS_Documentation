# CLI Overview

Welcome to Citros CLI. [Citros](https://citros.io/) serves as an innovative platform for executing ROS project simulations, automating integration, and conducting in-depth performance analysis.

The Citros CLI offers ROS 2 developers a seamless interface to launch multiple ROS simulations for a specific project with just a single command. Beyond setting static parameter values, it empowers users with the flexibility to utilize function objects. This means you can craft dynamic simulation environments where each execution produces unique parameter values, whether they're sourced from standard numpy functions or tailored via user-defined computations. Moreover, these operations can be executed offline without relying on any external dependencies.

Citros takes its capabilities a notch higher when the user logs in. Once logged in, users can tap into the full potential of Citros, ranging from running parallel simulations in the cloud to utilizing advanced data analysis tools for performance examination. Additionally, automatic report generation is a standout feature, aiding in effortless documentation of your work. Beyond these technical perks, logging in also paves the way for collaborative work, allowing you to engage and exchange ideas with team members.

For additional information, please refer to the Citros documentation. This will provide you with comprehensive insights and detailed instructions for effective usage of Citros in general and Citros CLI in particular, and their full suite of features.

We are dedicated to enriching your ROS project simulation experience, and this package is our contribution to that cause.

## Table of Contents
2. [Prerequisites](#prerequisites)
3. [Installation](#installation)
4. CLI Commands  (citros_data.md#citros_data_analysis.error_analysis.citros_data.CitrosData
   1. [Quick Start](/docs_cli/commands/quick_start)
   2. [init](/docs_cli/commands/init.md#init)
   3. [setup-ssh](/docs_cli/commands/setup-ssh.md#setup-ssh)
   4. [status](/docs_cli/commands/status.md#status)
   5.  [add-remote](/docs_cli/commands/add-remote.md#add-remote)
   6.  [commit](/docs_cli/commands/commit.md#commit)
   7.  [pull](/docs_cli/commands/pull.md#pull)
   8.  [push](/docs_cli/commands/push.md#push)
   9.  [diff](/docs_cli/commands/diff.md#diff)
   10. [checkout](/docs_cli/commands/checkout.md#checkout)
   11. [merge](/docs_cli/commands/merge.md#merge)
   12. [discard](/docs_cli/commands/discard.md#discard)
   13. [login](/docs_cli/commands/login.md#login)
   14. [logout](/docs_cli/commands/logout.md#logout)
   15. [list](/docs_cli/commands/list.md#list)
   16. [run](/docs_cli/commands/run.md#run)
   17. [docker-build](/docs_cli/commands/docker-build.md#docker-build)
   18. [docker-build-push](/docs_cli/commands/docker-build-push.md#docker-build-push)
5. [Citros Repository directory and file Structure](/docs_cli/structure/structure.md#citros-repository-file-structure) 
   1. [notebooks](/docs_cli/structure/notebooks.md#notebooks)
   2. [parameter setups](/docs_cli/structure/paramater_setups.md#parameter-setups)
   3. [reports](/docs_cli/structure/reports.md#reports)
   4. [runs](/docs_cli/structure/runs.md#runs)
   5. [simulations](/docs_cli/structure/simulations.md#simulations)
   6. [workflows](/docs_cli/structure/workflows.md#workflows)
   7. [project.json](/docs_cli/structure/project_json.md#projectjson)
   8. [settings.json](/docs_cli/structure/settings_json.md#settingsjson)
6. Citros Repository Configuration
    1. [Adding functions to parameter setup](/docs_cli/configuration/param_functions.md#adding-functions-to-parameter-setup)
        1. [How to Add Function Objects](/docs_cli/configuration/param_functions.md#how-to-add-function-objects)
        2. [Examples - numpy](/docs_cli/configuration/param_functions.md#examples---numpy)
        3. [Examples - user-defined](/docs_cli/configuration/param_functions.md#examples---user-defined)
        4. [Examples - full parameter_setup.json example](/docs_cli/configuration/param_functions.md#examples---full-parameter_setupjson-example)
        5. [Pitfalls and Gotchas](/docs_cli/configuration/param_functions.md#pitfalls-and-gotchas)
7. [User Templates](/docs_cli/user_templates.md#user-templates)


# Prerequisites

- [vscode](https://code.visualstudio.com/download)
- [Docker](https://www.docker.com/)
- [Python3](https://www.python.org/downloads/)
- [git](https://git-scm.com/)

# Installation
### option 1: without code:

        $ pip install citros 

### option 2: with code:

1. clone the repo:
    
        $ git clone git@github.com:lulav/citros_cli.git

2.  Within the cloned `citros_cli` folder, Install the package to a global bin folder:

        $ python3 -m pip install .
        $ source ~/.profile

### option 3: with code and soft links
If you are developing the citros_cli package itself, than the best practice is to create a `utils` directory under a ROS project, clone the repo into it, and install the package with soft links to dev environment. I.e. from your ROS project dir:

        $ mkdir utils && cd utils
        $ git clone git@github.com:lulav/citros_cli.git
        $ cd ..
        $ python3 -m pip install -e utils/citros_cli
    
3. Environment Variables. 
   
   `citros_cli` uses several environment variables, some of which you may change according to your needs, although for the most part, the defaults are likely to be what you want. Generally speaking, most of these are only used by developers of the package, and should not be used.

| ENV | Description | used in |
| --- | --- | --- |
| `CITROS_DOMAIN` | The main domain, defaults to `citros.io` | all packages |
| `CITROS_DIR` | Used by the citros cluster, do not use. | citros |
| `CITROS_SIM_RUN_DIR` | The directory under `.citros/runs` in which all simulation data will be saved (see [runs](docs_cli/structure/runs)). This can be handy, if your code needs to know this location in order to access some of the files, e.g. parameter setups. | citros |