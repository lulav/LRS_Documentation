## runs

The runs directory stores data and metadata about each run of your simulations. Its structure is as follows:

- Simulation Name: These directories are named after each of the simulations defined in the simulation files. For every simulation file that is run, a corresponding directory is created here. Each Simulation Name directory may include multiple Batch Name directories.
    - Batch Name: This directory holds a batch of simulation runs. A batch consists of multiple runs of the same simulation with different parameters.
        - Run ID: Each unique simulation run has its own directory, identified by a Run ID. Under this directory, there are several files and sub-directories:
            - `bag`: This sub-directory holds the recorded data from the simulation run. It includes:
                - bag_0.db3: This is a ROS bag file that contains all the messages that were sent during the simulation. The default bag format is `sqlite3` (hence the db3 postfix), but you may also use the `mcap` format. See [simulations](./simulations.md#simulations).
                - metadata.yaml: A file holding metadata information associated with the bag file.
            - `config`: This sub-directory contains YAML files (pkg1.yaml, pkg2.yaml, etc.) for each package in your ROS project, detailing the actual parameters used in the simulation. If you used any functions in your parameter setup, the values appearing here will be those that were evaluated according to the function you defined.
            - `msgs`: This sub-directory contains all the ROS msg files you may have in your project, each under yet another sub-directory with a name corresponding to the package the msg file belongs to.
            - `citros.log`: A standard log file that was active during the simulation run, documenting actions and events throughout the simulation.
            - `environment.json`: A file capturing a snapshot of your environment variables and Python packages at the time of the simulation run.
            - `info.json`: A JSON file containing general metadata about the run, such as batch ID, batch name, datetime of the run, user's Git commit and branch information, and Citros' Git commit and branch information, as well as a hash of the bag file.
            - `metrics.csv`: A CSV file recording system performance metrics during the simulation run, including CPU usage, total memory, available memory, used memory, and memory usage percentage.

These files collectively provide a comprehensive record of each simulation run, the conditions under which it was run, and the results it produced. This makes it easy to reproduce and understand the results of each simulation.

