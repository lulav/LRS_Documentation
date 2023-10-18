## push
The `push` command transfers all committed changes in your local Citros repository to the remote repository. Essentially, it acts as a wrapper for the `git push` command within the context of your Citros repo.

By employing the `push` command, you are synchronizing your local project modifications with the remote repository. This is crucial not only for backing up your work on the server but also for enabling seamless collaboration with other team members using the Citros platform.

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | Specifies the project's working directory. Defaults to `.`|
|`-d`, `--debug` | Sets the logging level to debug.|
|`-v`, `--verbose` | Enables verbose console output.|

### example

    $ citros push
    35461c6..d60a662

    Successfully pushed to branch `main`.
    $ citros push
    [up to date]

    Successfully pushed to branch `main`.

In the example above you can see that when there is a local commit to be pushed to the remote, `citros push` will push it and specify its commit hash. When running this command while already synched with the remote, you will be notified accordingly. 
