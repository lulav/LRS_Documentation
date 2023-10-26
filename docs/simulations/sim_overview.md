# Introduction

This page provides an overview of the main terms of the CITROS Simulation Workflow.

## Glossary

   |Term	|Description |
   |--|--|
   Project	|A user’s ROS-based project that includes a prepared Docker container file.
   Project's repository	|A user’s git repository, which must be initialized and synchronized with the local workspace prior to CITROS initialization.
   CITROS repository |A git repositroy hosted on the CITROS cloud. Manages all data related to CITROS, and synchronized with the Project's git repository during CITROS initialization.
   Simulation Batch |A set of one or more simulation runs, executed collectively with a designated configuration and parameter setup to efficiently analyze multiple scenarios or iterations.
   Simulation Run |A singular execution of a simulation model, performed with a specific set of parameters and conditions to analyze and observe system behavior in a specific scenario or iteration.

## Table of Contents
1. [Main Runs Page](/docs/simulations/sim_runs_page.md)
2. [Batch Page](/docs/simulations/sim_batch_page.md)
3. [Run Page](/docs/simulations/sim_run_page.md)
4. [Step-By-Step Tutorial](/docs/simulations/sim_step_by_step.md)