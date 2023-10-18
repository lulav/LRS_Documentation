## workflows

The `workflows` directory stores your JSON-formatted workflow files.

A flow.json file (e.g., `default_flow.json` which is auto-generated during `citros init`) is a user-crafted file used to automate and manage the flow of simulations in a citros repository. This file controls when the flow is triggered, which simulations are run, the post-processing analysis using Jupyter notebooks, and the recipients of the final reports. Here is a breakdown of its structure and content:

- `trigger`: This field specifies the event that initiates the flow. It is usually tied to some form of version control event, like a Git push, but can be configured according to the user's needs.
- `simulations`: This is an array of simulations to be run, specified as pairs of simulation name and the number of times to run them. For example, ["sim1", 17] means the simulation "sim1" will be run 17 times. Multiple simulations can be listed and each will be run the specified number of times.
- `notebooks`: This is a list of Jupyter notebooks used for post-processing analysis of the simulation results. For example, ["nb1.ipynb", "nb2.ipynb"] means these two notebooks will be run once the simulations complete, with the results used as their input data.
- `recipients`: This is a list of email addresses that will receive the reports generated from the notebooks' analysis.

The flow.json file helps to streamline and automate your citros repository by tying together simulation runs, data analysis, and report distribution into a single manageable file. You can customize it to suit the specifics of your project.
