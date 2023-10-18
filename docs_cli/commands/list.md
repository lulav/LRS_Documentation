## list
The `list` command displays all available simulation names. These names are derived from the filenames in the `simulations` folder within your Citros repository. Each of these files corresponds to an available launch file in your ROS project. For instance, if your ROS project contains a launch file named `foo.launch.py`, a corresponding simulation file named `simulation_foo.json` will be generated in your simulations folder.

### parameters:
parameter|description
|--|--|
|`-d`, `--debug` | Sets the logging level to debug.|
|`-v`, `--verbose` | Enables verbose console output.|

### example:

    $ citros list
    1. simulation_cannon_analytic
    2. simulation_cannon_numeric
