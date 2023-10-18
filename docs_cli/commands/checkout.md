## checkout
The `checkout` command lets you check out a different branch than the one your are currently on. It essentially wraps the `git checkout` command. If you have any uncommitted changes in your Citros working directory, you will be asked if you want to commit those changes. If you decline, the checkout will not take place, since Citros doesn't allow checking out while the working directory is dirty.

If the branch you're attempting to check out exists (locally or on the remote), it will be checked out. If it doesn't exist yet, you will be asked if you would like to create it. If you decline, the checkout will not take place.

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | Specifies the project's working directory. Defaults to `.`|
|`-d`, `--debug` | Sets the logging level to debug.|
|`-v`, `--verbose` | Enables verbose console output.|
|`-b`, `--branch` | The name of the branch to be checked out.|

### example

In the following example we checkout the branch `master` (not before confirming we want to commit the changes in our working directory), and then checkout the branch `main`. 

    $ citros checkout -b master
    Cannot checkout: there are uncommitted changes in your repo.
    Would you like to commit them? (y/n) y
    Checking out local branch master
    $ citros checkout -b main
    Checking out local branch main
