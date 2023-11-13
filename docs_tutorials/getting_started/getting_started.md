---
sidebar_position: 1
sidebar_label: 'Getting Started'
---
# Getting Started





## Installation



### Prerequisites


- [Docker](https://www.docker.com/)
- [Python3](https://www.python.org/downloads/)
- [git](https://git-scm.com/)


To install CITROS using pip, open terminal and write the following command:

    $ pip install citros 

### Environment Variables
   
   `citros_cli` uses several environment variables, some of which you may change according to your needs, although for the most part, the defaults are likely to be what you want. Generally speaking, most of these are only used by developers of the package, and should not be used.

| ENV | Description | used in |
| --- | --- | --- |
| `CITROS_DOMAIN` | The main domain, defaults to `citros.io` | all packages |
| `CITROS_DIR` | Used by the citros cluster, do not use. | citros |
| `CITROS_SIM_RUN_DIR` | The directory under `.citros/runs` in which all simulation data will be saved (see [runs](/docs_cli/structure/citros_structure#directory-runs)). This can be handy, if your code needs to know this location in order to access some of the files, e.g. parameter setups. | citros |

## Working Online with CITROS CLI

### Prerequisites
In addition to the prerequisites for working with the CITROS CLI offline, make sure:

- You have a working internet connection.
- You have signed up to [CITROS](https://citros.io) and have your login password.


Using the CITROS CLI online is very similar, from the user's standpoint, to using it offline. We still use the `init` and `run` commands, albeit with slight alterations or different consequences. In addition, there are a few more commands necessary to interact with the CITROS cloud.

### logging in

First and foremost, we need to log in:
```bash
$ citros login
email: example@lulav.space
Password: 
User logged in.
```

### ssh

Before we continue to initialize our project, there is one more prerequisite we need to check off that was not yet mentioned: setting up ssh communication with CITROS. Since this process only needs to be performed once per computer, and can be performed using the CITROS website, we will not delve into it here, except to mention that it can be performed via the CLI using the command:
```bash
citros setup-ssh
```
for further details, see the [CLI Documentation](https://citros.io/doc/docs_cli)


### Initialization

Assuming ssh communication has been set up, we can initialize our repository:
```bash
$ citros init
Checking internet connection...
Checking ssh...
Updating Citros...
Waiting for repo to be ready...
Citros repo successfully cloned from remote.
Creating and checking out a new local branch `main` that will track the remote branch.
Default branch of remote 'origin' set to: main
Citros successfully synched with local project.
You may review your changes via `citros status` and commit them via `citros commit`.
Initialized Citros repository.
```
As you can see, a lot more is happening when you initialize your repository while being logged in. We will not delve into all the details behind the scenes, but as always, feel free to roam through the [CLI Documentation](https://citros.io/doc/docs_cli) for further details.

:::note

 **Important**: the `.citros` directory can only be initialized once. If you try to run `init` while a `.citros` directory already exist in your project, you will get a response similar to this:
```bash
$ citros init
The directory /workspaces/cannon has already been initialized.
working remotely with [git@citros.io:lulav/cannon.git].
```
 If you want to reinitialize your CITROS repository, you'll have to first delete the current `.citros` directory.

 If you initialized the CITROS repository offline, and it doesn't exist on the remote server yet (i.e. it has not been initialized online by you or anyone else) - then rather than deleting the `.citros` directory, you can run:
 ```bash
 citros add-remote
 ```

 which will add the CITROS server as a remote for your CITROS repo on your behalf, and take care of a few other details that are handled when initializing while being logged in.

 At this point it is recommended you commit and push your changes to the remote by running:
 ```bash
 citros commit
 citros push
 ``` 
:::

### Building and Pushing a Docker Image

Now that our CITROS repository is initialized and synched with the CITROS remote, we have one more important thing we need to do before we can run our simulation on the cloud - we need to build a docker image of our ROS project, tag it with the current commit hash for the project, and upload it to CITROS. Sounds complicated? Not to worry - all this is accomplished by running a single command:
```bash
citros docker-build-push
``` 
:::note

As a prerequisite for this command, the working directory of your ROS project must be clean. If it isn't, simply commit your changes first.

:::

### Running on The Cloud

After running the previous command, CITROS has a docker image of your ROS project, so we can finally run the simulation on the cloud (notice the `-r` flag):
```bash
citros run -n "cloud_test" -m "running in the cloud!" -c 3 -r
? Please choose the simulation you wish to run: simulation_cannon_analytic
created new batch_id: aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeeeee. Running on Citros cluster. See https://citros.io/cannon/batch/aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeeeee.
```

The above command will run the `cannon_analytic` simulation 3 times on the CITROS cloud. By clicking the provided link, you can directly navigate to the *runs* tab on the CITROS website, and see your runs in action.