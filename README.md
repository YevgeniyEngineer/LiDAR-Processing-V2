# LiDAR-Processing-V2

A LiDAR processing pipeline based on ROS2 Humble node system, improvement to <https://github.com/YevgeniyEngineer/LiDAR-Processing>

## Enable Required Extensions in VSCode

```bash
code --install-extension --force ms-vscode-remote.remote-containers && \
code --install-extension --force ms-azuretools.vscode-docker
```

## Docker Setup

### Installation of Docker on Linux

Update the apt package index and install packages to allow apt to use a repository over HTTPS:

```bash
sudo apt-get update
sudo apt-get install --fix-missing apt-transport-https ca-certificates curl software-properties-common
```

Add Docker’s official GPG key

```bash
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
```

Set up the stable repository

```bash
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

Install Docker engine

```bash
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io
```

Start Docker engine and confirm installation

```bash
sudo systemctl start docker
sudo systemctl status docker
```

Enable Docker to Start at Boot

```bash
sudo systemctl enable docker.service
sudo systemctl enable containerd.service
```

### Steps to add to Docker Group

Create docker group:

```bash
sudo groupadd docker
```

Add your user to the Docker group:

```bash
sudo usermod -aG docker $USER
```

Restart your computer.

For the group change to take effect, you need to log out and then log back in. This is necessary because permissions are only re-evaluated by the system at login. Alternatively, you can use the following command to apply the changes without logging out:

```bash
newgrp docker
```

Set up a Docker account here: <https://hub.docker.com/signup> to be able to pull Docker images.

### Set Up Docker Credentials Helper

Install pass:

```bash
sudo apt-get install pass
```

Download and install credentials helper package:

```bash
wget https://github.com/docker/docker-credential-helpers/releases/download/v0.6.0/docker-credential-pass-v0.6.0-amd64.tar.gz && tar -xf docker-credential-pass-v0.6.0-amd64.tar.gz && chmod +x docker-credential-pass && sudo mv docker-credential-pass /usr/local/bin/
```

Create a new key:

```bash
gpg2 --gen-key
```

Initialize pass using the newly created key:

```bash
pass init "<Your Name>"
```

Open Docker config:

```bash
nano ~/.docker/config.json
```

Change the config to:

```json
{
    "auths": {
        "https://index.docker.io/v1/": {}
    }
}
```

Login to Docker using your own credentials (when prompted, enter your login and password):

```bash
docker login
```

### Enable X Server for Graphics Support

Allow Docker containers to display GUI applications on your host’s X server.

```bash
xhost +local:docker
```

or you might need to allow the root user on local connections:

```bash
xhost +local:root
```

### Install NVIDIA Container Toolkit

This step is optional, only applicable if you have NVIDIA GPU.

#### NOTE: IF YOU DON'T HAVE NVIDIA GPU, REMOVE "--runtime=nvidia" FROM .devcontainer.json

Add the Package RepositoriesOpen a terminal and add the NVIDIA package repositories:

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
```

Install NVIDIA DockerUpdate your package list and install the NVIDIA docker package:

```bash
sudo apt-get update
sudo apt-get install -y nvidia-docker2
```

Restart the Docker DaemonRestart the Docker daemon to apply the changes:

```bash
sudo systemctl restart docker
```

### Configure Docker to Use the NVIDIA Runtime

You can configure Docker to use the NVIDIA runtime by default so that every container you launch utilizes the GPU.

Edit or Create the Docker Daemon Configuration FileOpen or create the Docker daemon configuration file in your editor:

```bash
sudo nano /etc/docker/daemon.json
```

Add the Default RuntimeAdd or modify the file to include the default NVIDIA runtime:

```json
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
}
```

Restart the Docker service to apply these configuration changes:

```bash
sudo systemctl restart docker
```

## Build and Launch

Once you are inside of Devcontainer, you can build and launch the nodes. In the main directory:

```bash
./scripts/clean.sh
./scripts/build.sh
./scripts/launch.sh
```

## Example Output

### Point Cloud Data Format

Prior to processing point cloud is converted from unorganized to organized. The approximate ring partitioning scheme is used to cluster rings, approximating the natural Velodyne HDL-64E as closely as possible.

![ring-partitioning](https://github.com/YevgeniyEngineer/LiDAR-Processing-V2/blob/main/images/ring_partitioning.png)

### Segmentation

The implementation is based on ["Fast Ground Segmentation for 3D LiDAR Point Cloud Based on
Jump-Convolution-Process"](https://www.semanticscholar.org/paper/Fast-Ground-Segmentation-for-3D-LiDAR-Point-Cloud-Shen-Liang/01b8149e0ed6c5fe4932d961ff14ccca8f94ab47). The implementation is fairly rudimental, following most of the steps outlined in the original paper.

![segmentation](https://github.com/YevgeniyEngineer/LiDAR-Processing-V2/blob/main/images/segmentation.png)

### Clustering

The implementation is based on ["Curved-Voxel Clustering for Accurate Segmentation of 3D LiDAR Point Clouds with Real-Time Performance"](https://ieeexplore.ieee.org/document/8968026).

TO BE IMPLEMENTED.
