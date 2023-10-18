## init
The `init` command is used to initialize a Citros repository. Depending on the user's login status, this behavior varies. For logged-out users, the project initializes locally. However, logged-in users will have the `.citros` directory cloned from the Citros remote repository. If it's a new project, an empty project will be cloned.

The initialization process involves creating a `.citros` directory within your ROS project directory and generating several files and folders therein. These files are set up to allow you to run a simulation of your project with default configurations and settings. You can tailor your Citros repository to your specific needs by manually modifying these files (see the Project Configuration section for more details).

**Note:** the initialization process will also make sure that within your Citros repo, you are working on a branch whose name is the same as the current branch in your ROS project. It will do so by checking it out (and possibly creating such a branch if it does not already exist).

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | Specifies the project's working directory. Defaults to `.`|
|`-d`, `--debug` | Sets the logging level to debug.|
|`-v`, `--verbose` | Enables verbose console output.|
|`-project_name` <proj_name> | Optional name for the project. Defaults to the last folder in the path of *dir*|

### examples

example 1 - initializing while logged out:

    $ citros init
    User is not logged in. Initialzing Citros locally.
    Intialized Citros repository.

example 2 - initializing while logged in:

    $ citros init
    Checking internet connection...
    Checking ssh...
    Updating Citros...
    Waiting for repo to be ready...
    Citros repo successfully cloned from remote.
    Citros successfully synched with local project.
    You may review your changes via `citros status` and commit them via `citros commit`.
    Intialized Citros repository.

Note: The init command can only be executed with effect once per project. If you attempt to initialize an existing Citros repository, you will be notified that the action is redundant, and no changes will be made. Example:

    $ citros init
    The directory /workspaces/cannon has already been initialized.
    No remotes found and user is not logged in. Working offline.

To re-initialize an existing Citros repository, you must first delete the existing .citros directory for your project.
