## diff
The `diff` command presents you with a detailed description of all differences between the latest commit and your working directory. New lines will be colored in green, and deleted lines will be colored in red.

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | Specifies the project's working directory. Defaults to `.`|
|`-d`, `--debug` | Sets the logging level to debug.|
|`-v`, `--verbose` | Enables verbose console output.|

### example:
    $ citros diff
    diff --git a/simulations/simulation_cannon_analytic.json b/simulations/simulation_cannon_analytic.json
    index e7c823f..178c95b 100644
    --- a/simulations/simulation_cannon_analytic.json
    +++ b/simulations/simulation_cannon_analytic.json
    @@ -5,7 +5,7 @@
            "file": "cannon_analytic.launch.py",
            "package": "scheduler"
        },
    -    "timeout": 60,
    +    "timeout": 42,
        "GPU": 0,
        "CPU": 2,
        "MEM": "265MB",
