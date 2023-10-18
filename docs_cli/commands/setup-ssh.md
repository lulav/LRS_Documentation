## setup-ssh
The `setup-ssh` command sets up SSH keys for secure communication with the remote Citros repository.

Setting up your ssh keys can be done in several different ways. You can do it manually by yourself, following the instructions on the [citros.io](https://citros.io) website, or you can use the `setup-ssh` command to automate this process.

When using `setup-ssh`, you may run it directly on your computer, in which case you will only ever need to run it once. This, of course, means you'll need to install the citros-cli directly on you computer, rather than inside a dev-container. 

If you'd rather avoid this, you can also run `setup-ssh` inside a dev container, but the price in that case, is that you'll have to run it once for each dev-container you use (and again if you rebuild the dev-container). Also, since you are prompted to give a unique title for the ssh key that will be generated, you will have to do so every time you run `setup-ssh`. 

In any case, you may view (and possibly delete) your keys in your profile settings on the [citros.io](https://citros.io) website. 

**Note:** this command *may* append some bash commands to the end of any of the following user profile files, if they exist in the user's home directory: `~/.bashrc` , `~/.bash_profile`, `~/.zprofile`. 

### prerequisites:
- user must be logged in (using `citros login`).

### parameters:
parameter|description
|--|--|
|`-d`, `--debug` | Sets the logging level to debug.|
|`-v`, `--verbose` | Enables verbose console output.|
