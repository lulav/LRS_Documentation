## commit
The `commit` command captures all modifications to your local Citros repository in a snapshot, essentially serving as a wrapper for the `git commit` command, but tailored to your Citros repository.

By executing this command, you essentially save the current state of your project, allowing you to keep track of your progress, revert changes, and even collaborate more effectively. This forms an integral part of managing and controlling the version history of your Citros repository.

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | Specifies the project's working directory. Defaults to `.`|
|`-d`, `--debug` | Sets the logging level to debug.|
|`-v`, `--verbose` | Enables verbose console output.|
|`-m`, `--message` | Commit message|

### example:

    $ citros commit -m "added an awesome feature"