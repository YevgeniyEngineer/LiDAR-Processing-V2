# LiDAR-Processing-V2

A LiDAR processing pipeline based on ROS2 Humble node system, improvement to <https://github.com/YevgeniyEngineer/LiDAR-Processing>.

See execution times captured with `gprof` in [analysis.txt](./analysis.txt).

```bash
git submodule init
git submodule update --init --recursive
```

## List of Implemented Algorithms and Data Structures

- KDTree
- Dynamic Radius Outlier Removal (DROR) filter
- Segmentation based on Jump Process Convolution and Ring-Shaped Elevation Conjunction Map
- Curved Voxel Clustering
- Andrew's Monotone Chain Convex Hull
- Shamos Algorithm for finding antipodal vertex pairs of convex polygon
- Bounding Box fitting using Rotating Calipers and Shamos acceleration
- Bounding Box fitting using Principal Component Analysis
- Rudimental vehicle classification based on shape matching

![segmentation-teaser](https://github.com/YevgeniyEngineer/LiDAR-Processing-V2/blob/main/images/segmentation_teaser.gif)

![clustering-teaser](https://github.com/YevgeniyEngineer/LiDAR-Processing-V2/blob/main/images/clustering_teaser.gif)

## Example Devcontainer and VSCode Tasks

[![Watch the video](https://img.youtube.com/vi/AN4WgETAy6A/0.jpg)](https://www.youtube.com/watch?v=AN4WgETAy6A)

## Build Dependencies

For ROS2 installation, if you prefer not using Devcontainer, follow <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#ubuntu-debian-packages>.

I included Devcontainer setup instructions in `Docker Setup` section below.

- `build-essential`
- `cmake`
- `ros-humble-desktop`
- `libpcl-dev`
- `libopencv-dev`

## Build and Launch

Once you are inside of Devcontainer, you can build and launch the nodes. In the main directory:

```bash
./scripts/clean.sh
./scripts/build.sh
./scripts/launch.sh
```

## GDB Debugging

It is recommended to setup GDB dashboard: <https://github.com/cyrus-and/gdb-dashboard>.

By convention, I assigned one node per package, so the node name matches its corresponding package name. This was done to simplify the launch scripts and setting up Tasks with VS Code.

```bash
# Build a single package in Debug mode
./scripts/build.sh -d processor

# Launch an individual node
./scripts/launch.sh dataloader
./scripts/launch.sh processor
./scripts/launch.sh rviz2

# Launch a GDB Debugger VS Code task
# Ctrl + Shift + P -> Tasks: Run Task -> Launch ROS2 Node with GDB
# When prompted, enter package name processor
```

## Point Cloud Data Format

Prior to processing point cloud is converted from unorganized to organized, however this is strictly not necessary because I templated segmentation function to accept unorganized point cloud too. The approximate ring partitioning scheme is used to cluster rings, approximating the natural Velodyne HDL-64E scan pattern as closely as possible. If the cloud is provided in the unorganized format, the height index of the image will be determined from linear point mapping, considering vertical field of view of the Velodyne sensor.

![ring-partitioning](https://github.com/YevgeniyEngineer/LiDAR-Processing-V2/blob/main/images/ring_partitioning.png)

## Segmentation

The implementation is based on ["Fast Ground Segmentation for 3D LiDAR Point Cloud Based on
Jump-Convolution-Process"](https://www.semanticscholar.org/paper/Fast-Ground-Segmentation-for-3D-LiDAR-Point-Cloud-Shen-Liang/01b8149e0ed6c5fe4932d961ff14ccca8f94ab47). The implementation follows most of the steps outlined in the original paper, except I added an intermediate RANSAC filter to correct erroneous classification close to the vehicle, which are caused by either pitch deviation of the vehicle, uncertainty in sensor mounting position, or reflective LiDAR artifacs, present below the ground surface.

![segmentation](https://github.com/YevgeniyEngineer/LiDAR-Processing-V2/blob/main/images/segmentation.png)

## Clustering

The implementation is based on ["Curved-Voxel Clustering for Accurate Segmentation of 3D LiDAR Point Clouds with Real-Time Performance"](https://ieeexplore.ieee.org/document/8968026).

![clustering_1](https://github.com/YevgeniyEngineer/LiDAR-Processing-V2/blob/main/images/clustering_1.png)

![clustering_2](https://github.com/YevgeniyEngineer/LiDAR-Processing-V2/blob/main/images/clustering_2.png)

## Polygonization

![polygonization](https://github.com/YevgeniyEngineer/LiDAR-Processing-V2/blob/main/images/polygonization_teaser.gif)

Obstacle clusters are further simplified by calculating the outer contour (polygon) of the point cluster.

There are three main types of simple polygons considered:

- Convex hull (Andrew's Monotone Chain) [Implemented]
- Oriented bounding box (Rotating Calipers with Shamos acceleration and Principal Component Analysis) [Implemented]
- Concave hull (X-Shape Concave Hull) [Not implemented]

The simplified obstacle contours can be used for:

- Filtering obstacles by size
- Obstacle tracking
- Collision detection
- Dynamic path planning

I attempted to extract vehicles from the scene with a reasonable success using classical processing techniques, without relying on neural networks.
However, it seems there are limitations on how well vehicles can be extracted from the scene.

![classification_of_vehicles](https://github.com/YevgeniyEngineer/LiDAR-Processing-V2/blob/main/images/vehicle_classification.png)

## Code Profiling Procedure

To enable code profiling, add the following lines in `CMakeLists.txt`:

```bash
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
```

To generate the profiler analysis information:

```bash
gprof ./install/processor/lib/processor/processor ./gmon.out > analysis.txt
```

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

## Citation

If you use this software, please cite it as follows:

```bibtex
@software{simonov_lidar_pipeline_2024,
  author = {Simonov, Yevgeniy},
  title = {{LiDAR Processing Pipeline}},
  url = {https://github.com/YevgeniyEngineer/LiDAR-Processing-V2},
  version = {0.2.0},
  date = {2024-06-09},
  license = {GPL-3.0}
}
```
