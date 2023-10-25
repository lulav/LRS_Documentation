# CLI Installationn

The following document will guide you through CITROS CLI installation.

## Prerequisites

- [vscode](https://code.visualstudio.com/download)
- [Docker](https://www.docker.com/)
- [Python3](https://www.python.org/downloads/)
- [git](https://git-scm.com/)

## Installation

To install CITROS using pip, open terminal and write the following command:

    $ pip install citros 

## Environment Variables
   
   `citros_cli` uses several environment variables, some of which you may change according to your needs, although for the most part, the defaults are likely to be what you want. Generally speaking, most of these are only used by developers of the package, and should not be used.

| ENV | Description | used in |
| --- | --- | --- |
| `CITROS_DOMAIN` | The main domain, defaults to `citros.io` | all packages |
| `CITROS_DIR` | Used by the citros cluster, do not use. | citros |
| `CITROS_SIM_RUN_DIR` | The directory under `.citros/runs` in which all simulation data will be saved (see [runs](../structure/citros_structure.md#directory-runs)). This can be handy, if your code needs to know this location in order to access some of the files, e.g. parameter setups. | citros |

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';