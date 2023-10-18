## docker-build-push
The `docker-build-push` command is used to construct a Docker image of your ROS project and upload it to Docker Hub. This image encapsulates your project's environment, facilitating the portability and reproducibility of your simulations across different systems. 

Two tagged images will be built and pushed: `latest` and the ROS project's latest commit hash, so that it is archived in the docker registry.

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

### example
    $ citros docker-build-push
    Building Docker image...
    => building with "default" instance using docker driver
    => ...
    Done.
    Pushing Docker image...
    The push refers to repository [us-central1-docker.pkg.dev/citros/lulav/cannon]
    ea97705925b1: Pushed 
    0d42db2cff87: Preparing 
    3752398ae296: Layer already exists 
    ...
    latest: digest: sha256:b5d109f83c1dbbaf97d918e721889988210d9bc1a91f3ecde884fbc394bcca1c size: 5136
    The push refers to repository [us-central1-docker.pkg.dev/citros/lulav/cannon]
    ea97705925b1: Layer already exists 
    5deba67bdae6: Layer already exists 
    ... 
    7f858865b89b41f493d1197c3329c0214996a625: digest: sha256:b5d109f83c1dbbaf97d918e721889988210d9bc1a91f3ecde884fbc394bcca1c size: 5136
    Done.