## simulations

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