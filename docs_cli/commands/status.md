## status
The `status` command first syncs any changes in your ROS project with your Citros repository and than retrieves the current state of your Citros repository. Essentially, it acts as a wrapper for the `git status` command specifically for your Citros repository.

This command provides a quick and concise overview of the changes made to your project, giving you insights into tracked, modified, and staged files.

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | Specifies the project's working directory. Defaults to `.`|
|`-d`, `--debug` | Sets the logging level to debug.|
|`-v`, `--verbose` | Enables verbose console output.|

### example

In the example below, we employ the `status` command to gain insight into the condition of our Citros repository. This becomes particularly beneficial when there's a divergence between your local and remote branches—like when the remote branch received updates you haven't pulled yet, while you've committed local changes still awaiting a push to the remote.:

    $ citros status
    On branch main
    Your branch and 'origin/main' have diverged,
    and have 1 and 4 different commits each, respectively.

    nothing to commit, working tree clean
    $ citros pull
    $ citros status
    On branch main
    Your branch is ahead of 'origin/main' by 2 commits.

    nothing to commit, working tree clean
    $ citros push
    $ citros status
    On branch main
    Your branch is up to date with 'origin/main'.

    nothing to commit, working tree clean