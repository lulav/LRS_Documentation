---
sidebar_position: 150
sidebar_label: 'Galactic Orbits'
---

# Globular Star Cluster Orbit Simulations

## Overview

CITROS offers multiple advantages that can streamline your workflow:

- a seamless pipeline from setting parameters and running simulations to analyzing output results with the [**citros_data_analysis**](https://citros.io/doc/docs_data_analysis/) package, and generating the final report;
- an opportunity to run multiple simulations simultaneously;
- a convenient and well-organized data storage system: you can store all your simulation data in the cloud, track their status via the CITROS website, and access this data from different devices;
- two options for organizing your work: manage and run simulations through a [web interface](https://citros.io/doc/docs/) or use the [command line](https://citros.io/doc/docs_cli).

All these features make CITROS exceptionally useful and user-friendly for projects involving large data sets, diverse simulation scenarios, and complex analyses, which are common in various scientific research fields.

The current project is dedicated to calculations of orbits of the globular star cluster for five different values of the Galactic disk mass: 95, 97.5, 100, 102.5 and 105 billions of the Sun masses.

Globular star clusters are gravitationally bound, dense and rich aggregations of stars, that can be found nearly in every galaxy, including our own Milky Way. In comparison with another type of star clusters, the open clusters, globular clusters are populated with older stars and can have up to millions of members. Another notable difference is that they are not part of the disk component but belong to the halo. This means they can be located far from both the thin and thick disks of the Galaxy, which have thicknesses of about 300 pc and 2.6 kpc, respectively.

![figNGC6316](img/NGC6316.png "NGC6316")

*globular star cluster NGC 6316, Atlas Image [or Atlas Image mosaic] obtained as part of the Two Micron All Sky Survey (2MASS), a joint project of the University of Massachusetts and the Infrared Processing and Analysis Center/California Institute of Technology, funded by the National Aeronautics and Space Administration and the National Science Foundation*.

## Prerequisites
To calculate orbits the Python package [GalOrb](https://github.com/ChemelAA/GalOrb-Package) was adopted. It is automatically installed when the docker development container of the project is build. The orbits are calculated in a non-axisymmetric gravitational potential, using an adopted model of the Galaxy with four components: disk, spheroid, dark-matter halo, and a bar. Details on this package, as well as parameters required for simulations, can be found in the article [Globular Clusters: Absolute Proper Motions and Galactic Orbits](https://link.springer.com/article/10.1134/S1990341318020049), [arXiv](https://arxiv.org/pdf/1804.07086.pdf).

If you are working without docker please check other dependencies in Dockerfile in [.devcontainer](https://github.com/citros-garden/gal_orbits/tree/main/.devcontainer) folder.

## Table of Contents
- [Installation](#installation)
- [Workspace Overview](#workspace-overview)
    - [Input Parameters](#input-parameters)
    - [Source Code and Launch File](#source-code-and-launch-file)
    - [Output of the Simulation](#output-of-the-simulation)
- [CITROS Initialization](#citros-initialization)
- [Scenario](#scenario)
    - [Parameter setup](#parameter-setup)
    - [Simulation setup](#simulation-setup)
- [Running the scenario using CITROS](#running-the-scenario-using-citros)
- [Results](#results)

## Installation
```bash
git clone git@github.com:citros-garden/gal_orbits.git
```

## Workspace Overview
### Input Parameters

Parameters of the simulation with their default values are listed in `srs/gal_orbits/config/params.yaml` file:

parameter | description | default
|--|--|--
publish_freq| frequency of publishing | 10
rh| heliocentric distance of the object (in kpc) | 0
lon| galactic longitude of the object (in degrees) | 0
lat| galactic latitude of the object (in degrees) | 0
vr| heliocentric radial velocity of the object (in 100 km/s) | 0
pmra| proper motion in right ascension (in mas/year) | 0
pmde| proper motion in declination | 0
t0| starting time of calculation (in units of 10^7 years) | 0
tf| final time of calculation (in units of 10^7 years) | 10
M_disc| mass of the disc, in Msun * 10^9, by default, 10^11 Msun | 100
M_sph| mass of the spherical component of the Galaxy, in Msun * 10^9, by default, 3 * 10^10 Msun | 300
reverse| if 'True', set backward direction of time, by default, direction is forward | 'False'
rtol| relative value of the error of the numerical integration scheme, affects the output number of messages | 1e-9
atol| absolute value  of the error of the numerical integration scheme, affects the output number of messages | 1e-9

The table with the parameters for the 115 globular clusters may be found in article, mentioned above: [Globular Clusters: Absolute Proper Motions and Galactic Orbits](https://link.springer.com/article/10.1134/S1990341318020049), [arXiv](https://arxiv.org/pdf/1804.07086.pdf).

### Source Code and Launch File
The source code is located in `srs/gal_orbits/gal_orbits/`: `gal_orbits.py` - written in Python script that calculates orbits of the globular clusters and `__init__.py` file.

The launch file is located in `srs/gal_orbits/launch/gal_orbits.launch.py`.

### Output of the Simulation

The output of the simulation is an array of 11 variables:

- t - time coordinate (in units of 10^7 years),
- R - distance from the galactic axis (in kpc),
- Vr - dR/dt, radial component of the velocity (in 100 km/s),
- fi - the position angle relative to Sun direction, counting clockwise if seen from North Galactic Pole (in radians),
- Vfi - R*d(fi)/dt, tangential velocity (in 100 km/s),
- z - vertical distance from the galactic plane (in kpc),
- Vz - dz/dt, vertical velocity (in 100 km/s),
- E - total energy (in (100 km/s)^2),
- C - angular momentum (in 100 kpc*km/s),
- xg - R*cos(fi), X galactocentric coordinates (in kpc),
- yg - R*sin(fi), Y galactocentric coordinates (in kpc)

## CITROS Initialization

To start working with CITROS you need to install CITROS CLI package, log in, set ssh key and initialize the `.citros` repository. To do this please follow [Getting Started tutorial](https://citros.io/doc/docs_tutorials/getting_started/).

## Scenario
### Parameter setup

Let's check how the mass of the Galactic disk affects the orbit of the globular clusters. For this purpose we can run several simulations with different values of `M_disc` parameter.

Parameters are listed in file `.citros/parameter_setups/default_param_setup.json`. For example, to set simulation parameters for cluster **NGC 6316** and calculate its orbits with 5 slightly different masses of the Galaxy disk (95, 97.5, 100, 102.5 and 105 billions of the Sun masses), the following setup may be used:

```js
{
    "packages": {
        "gal_orbits": {
            "gal_orbits": {
                "ros__parameters": {
                    "publish_freq": 10.0,
                    "rh": 11.5,
                    "lon": 357.18,
                    "lat": 5.76,
                    "vr": 0.715,
                    "pmra": -4.52,
                    "pmde": -3.70,
                    "t0": 0.0,
                    "tf": 20.0,
                    "M_disc": {
                        "function": "my_func.py:return_next_value",
                        "args": [[95.0, 97.5, 100.0, 102.5, 105.0]]
                    },
                    "M_sph": 30.0,
                    "reverse": "False",
                    "rtol": 1e-8,
                    "atol": 1e-8
                }
            }
        }
    }
}
```
The parameters are taken from the article [Globular Clusters: Absolute Proper Motions and Galactic Orbits](https://link.springer.com/article/10.1134/S1990341318020049), [arXiv](https://arxiv.org/pdf/1804.07086.pdf).

Function *my_func.py:return_next_value* returns the next listed in `args` value for each subsequent simulation. Function should be written in file `.citros/parameter_setups/functions/my_func.py`:

```python
def return_next_value(arr, citros_context):
    return arr[citros_context['run_id']]
```
Learn more about parameter setup and defining custom functions in [Directory parameter_setups](https://citros.io/doc/docs_cli/structure/citros_structure/#directory-parameter_setups) and [Adding Functions to Parameter Setup](https://citros.io/doc/docs_cli/configuration/config_params) pages.

### Simulation setup

In `.citros/simulations/simulation_gal_orbits.json` you can define parameter setup files, launch files, memory to use and so on, please look in [Directory simulations page](https://citros.io/doc/docs_cli/structure/citros_structure#directory-simulations) for more information.

## Running the scenario using CITROS

After adjusting [parameter](#parameter-setup) and [simulation](#simulation-setup) setups, you need to commit and push your changes and also build and push a docker image. To do this please continue follow the [Getting Started tutorial](https://citros.io/doc/docs_tutorials/getting_started/).

And now it's time to run the simulation in the cloud! Do the following command to run simulation 5 times in the cloud and assign the results to a "galactic orbits" batch:

```bash
citros run -n "galactic orbits" -m "first run" -c 5 -r
```

Explore more ways of running scenarios on [Getting Started page](https://citros.io/doc/docs_tutorials/getting_started/).

## Results
Now when your simulation is complete, you're ready to check the results! Explore the notebooks in [`citros_template/notebooks`](https://github.com/citros-garden/gal_orbits/tree/main/citros_template/notebooks). There, you'll find examples prepared using the [citros_data_analysis package](https://citros.io/doc/docs_data_analysis) on how to query, analyze and present results. Feel free to use them or create your own!

```python
from citros_data_analysis import data_access as da
citros = da.CitrosDB()

# query data using citros_data_analysis package, get pandas.DataFrame
F = citros.batch('galactic orbits').topic('/gal_orbits').data(['data.data[9]', 'data.data[10]'])

# change, calculate, explore your data - take full advantage of working with pandas python package
F.rename({'data.data[9]': 'xg', 'data.data[10]': 'yg'}, axis = 1, inplace = True)

# plot with citros_data_analysis
fig, ax = citros.plot_graph(F, 'xg', 'yg', '-', set_x_label='x, kpc', set_y_label='y, kpc', title='Projection onto the Galactic plane, NGC 6316')

# add additional information on the graph: position of the Sun and start of the simulation
ax.plot(0, 0, 'y*', markersize = 12)
ax.plot(F['xg'][F['sid']==0].iloc[0], F['yg'][F['sid']==0].iloc[0], 'bo')
```

![fig_ngc6316_xy](img/fig_ngc6316_xy.png "fig_ngc6316_xy")

*Projection of the globular Galactic cluster NGC 6316's orbits onto the Galactic plane over the next 200 million years. Galactocentric coordinates 'xg' and 'yg' were derived from orbit simulations in a non-axisymmetric gravitational potential, considering five scenarios with slightly varying Galaxy disk masses (95, 97.5, 100, 102.5 and 105 billion solar masses). The yellow star represents the Sun, located at coordinates (0, 0). The simulation starts at the point marked by a blue circle.*