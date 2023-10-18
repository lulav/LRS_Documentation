---
slug: Drone
title: Drone
authors: [iftahnaf]
tags: [CITROS]
---

# Drone Tutorial

add cool image here

**Table of contents**

1. [General Information](#general-Information-🌐)
2. [Installation](#installation-🛫)
3. [CITROS Integration](#citros-integration-🛸)
    1. [Configuring The Project](#configuring-the-project-⚙️)



## General Information 🌍

This repository contains an example of a ROS 2 node which communicate with a drone using PX4 and ROS 2.
Communication via uXRCE-DDS (PX4 version `1.14.0`).  
The repository contains launch file which launch:
1. Gazebo simulation (headless / gui options available).
2. A PX4 instans which control the simulated drone.
3. DDS agent for ROS 2 - PX4 communication.
4. An Offboard node which sends setpoints for the control system.

## Installation 🛫

Clone the repository from Github:

                https://github.com/citros-garden/drone.git

Then open the repository in VSCode's `devcontainer` with `reopen in container option`.
The [Dockerfile](.devcontainer/Dockerfile) contains all the necessary dependencies for the project, and the 
[install](.devcontainer/install.sh) script will clone PX4-Autopilot and build the firmware, along with building ROS 2 workspace.

## CITROS Integration 🛸
To use all the powerfull CITROS features usage requires CITROS installation:  
(from the instructions on the CITROS CLI [GitHub page](https://github.com/lulav/CITROS_cli))

### Configuring the project

After all the prerequisites done, we can start configuring our project. Open the project's repository in `VSCode` and reopen the project in a the VSCode's `devcontainer`.

### Parameters