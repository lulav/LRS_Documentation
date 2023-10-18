## pull
The `pull` command fetches from and integrates with another Citros repository or a local branch. Essentially, it acts as a wrapper for the `git pull` command within the context of your Citros repo.

**Note:** if there conflicts between your local copy and the remote copy that cannot be resolved automatically, than a manual merge will have to take place. Not to worry - Citros makes this process user-friendly - see [Merge](./merge.md#merge) for details.

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | Specifies the project's working directory. Defaults to `.`|
|`-d`, `--debug` | Sets the logging level to debug.|
|`-v`, `--verbose` | Enables verbose console output.|

### example

    $ citros pull
