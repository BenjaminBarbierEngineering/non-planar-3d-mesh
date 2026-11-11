## Development containers

This folder contains two Dev Container configurations. Both provide the same
development environment and VS Code extensions, but differ in how the
container image is obtained.

The build image uses Docker Hub's official `ubuntu:jammy-20240111` image, a
dated Ubuntu 22.04 LTS (Jammy Jellyfish) tag. See the
[official Ubuntu image](https://hub.docker.com/_/ubuntu) and
[Ubuntu 22.04 (Jammy) releases](https://releases.ubuntu.com/jammy/).
Both configurations mount the host `.bashrc` at `/home/vscode/.bashrc`.

### Build the image through VS Code

Use this option to let the Dev Containers extension build the image for you:

1. Open the repository in VS Code with the Dev Containers extension installed.
2. Run **Dev Containers: Reopen in Container** from the Command Palette.
3. If prompted to choose a configuration, select **build-image**.

The extension builds the image using
[`build-image/Dockerfile`](./build-image/Dockerfile) and then opens the
development container. If you change the Dockerfile, rebuild the container
using **Dev Containers: Rebuild Container**.

### Build and tag the image yourself

Use this option to build a named image once, then use it with the
`sdk-image` configuration:

1. From a terminal, build the image from the directory containing the
   Dockerfile:

   ```sh
   cd .devcontainer/build-image
   docker build -t non-planar-3d-mesh-dev:latest .
   ```

   Replace `non-planar-3d-mesh-dev:latest` with the image name and tag you
   want to use.

2. Edit [`sdk-image/devcontainer.json`](./sdk-image/devcontainer.json) and
   replace the value of `"image"` with the same image name and tag. For
   example:

   ```json
   "image": "non-planar-3d-mesh-dev:latest"
   ```

3. Open the repository in VS Code, run **Dev Containers: Reopen in Container**,
   and select **sdk-image** if prompted.

When you update the Dockerfile, rebuild the image with the same tag before
reopening the `sdk-image` container. If you use a different tag, update the
`"image"` value to match it.
