## add-remote
The `add-remote` command associates a remote Citros repository, named `origin`, with your local repository. This remote repository is hosted on the Citros servers.

### prerequisites:
`citros setup-ssh` has already been run.

**Important**: If you execute `citros init` while logged in, the `add-remote` command will automatically run in the background, making a direct call unnecessary. However, if you initially ran `citros init` while logged out and later decide to work with the online Citros system (e.g., running commands like `citros push`), you will need to manually run the `add-remote` command.

Furthermore, to ensure secure communication with the server, the `setup-ssh` command should be executed before running add-remote.

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | Specifies the project's working directory. Defaults to `.`|
|`-d`, `--debug` | Sets the logging level to debug.|
|`-v`, `--verbose` | Enables verbose console output.|