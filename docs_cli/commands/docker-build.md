## docker-build
The `docker-build` command is used to construct a Docker image of your ROS project. This image encapsulates your project's environment, facilitating the portability and reproducibility of your simulations across different systems.

### prerequisites
If you are working inside a dev-container, make sure that the `docker-in-docker` feature is enabled in your project's `devcontainer.json`, i.e.:

    "features": {
		"ghcr.io/devcontainers/features/docker-in-docker:2": {
			"version": "latest",
			"moby": true
		}
	}

### parameters:
parameter|description
|--|--|
|`-dir` <folder_name> | Specifies the project's working directory. Defaults to `.`|
|`-d`, `--debug` | Sets the logging level to debug.|
|`-v`, `--verbose` | Enables verbose console output.|
|`-n`, `--image_name` | The requested image name (e.g. the project name). Defaults to the last folder in the path of dir |
|`-t`, `--tag` | the requested tag name for the image. Defaults to `latest`|

### example

    $ citros docker-build
    Building Docker image...
    => building with "default" instance using docker driver
    => ...
    Done.