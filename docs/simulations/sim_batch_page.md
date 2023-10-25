<!-- # Simulation Run
## Introduction
## Simulation Run Table
the explanation you wrote
## Simulation Events
short explanation
### Time
### Type...

-->
# Batch Page Overview

## Introduction
The Batch page provides a comprehensive dashboard that allows users to monitor and manage batch simulation runs, offering detailed information about each run and overall batch status, while also providing insights into the development context in which the simulations were run.

![png](img/batch0.png "Batch page overview")

## Simulation Info

The top of this page contains the Batch Run Info for quick review of the context and purpose of the current batch simulation. It includes Repository Name, Batch Name, Sumulation Name, Run number and user-defined message. The Batch ID is located on the right.

![png](img/batch1.png "Simulation Info")

## Simulation Run Table

The Simulation Runs Table is located in the center of the page. You can observe it to monitor the status and progress of individual simulations in the batch. Click on specific simulation IDs or rows to navigate to detailed run page. This table has 4 columns:

* Run number
* [Last Event of the Run](https://citros.io/doc/docs/simulations/sim_batch_runs)
* [Status](https://citros.io/doc/docs/simulations/sim_batch_run)
* Timestamp of creation

![png](img/batch2.png "Simulation Run Table")

## Batch Information

Review the Info section (on the right) to gain insights into the batch's status, progress, and associated development context (git branch, commit ID). The Information section provides:

* Status: general batch status. Depends on statuses of all related simulations. See [Batch Statuses](#batch-status) for additional info.
* Timestamp of creation
* Progress: the number of Runs in DONE status in relation to the total number of Runs in the Batch
* Created by: nickname of the user, who created this Batch
* Branch
* Commit: commit ID
* CITROS commit: CITROS commit ID
* Image: link to the used image
* [Data status](https://citros.io/doc/docs/simulations/sim_runs_page)
* Link to the data of the Batch.

![png](img/batch3.png "Batch Information")

Use this information for troubleshooting, understanding the developer context, and monitoring database status.

### Batch Status

The Batch may exist in one of the following states: 
* SCHEDULE: The first stage of the batch run. Batch runs are initializing
* RUNNING: All simulations are running
* TERMINATING: the simulaitons theirself finished, but other operational proccesses (e.g. uploading data to the Database) need to finish
* DONE: All related simulation runs are in DONE state
* All related simulation runs are in DONE, but at least one of them is in ERROR status.

The status of the Batch is contingent upon the cumulative statuses of all simulations associated with said batch.