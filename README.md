# LiDAR-Processing-V2
A LiDAR processing pipeline based on ROS2 Humble node system, improvement to https://github.com/YevgeniyEngineer/LiDAR-Processing

## Enable Required Extensions in VSCode
```bash
code --install-extension --force ms-vscode-remote.remote-containers && \
code --install-extension --force ms-azuretools.vscode-docker
```

## Docker Setup

#### Update the apt package index and install packages to allow apt to use a repository over HTTPS:
```bash
sudo apt-get update
sudo apt-get install --fix-missing apt-transport-https ca-certificates curl software-properties-common
```

#### Add Docker’s official GPG key:
```bash
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
```

#### Set up the stable repository:
```bash
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

#### Install Docker Engine
```bash
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io
```

#### Start Docker Engine and Confirm Installation
```bash
sudo systemctl start docker
sudo systemctl status docker
```

#### Enable Docker to Start at Boot
```bash
sudo systemctl enable docker.service
sudo systemctl enable containerd.service
```

#### Steps to add to Docker Group

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

Set up a Docker account here: https://hub.docker.com/signup to be able to pull Docker images.

#### Set Up Docker Credentials Helper

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

Login to Docker using your own credentials:
```bash
docker login
```

#### Enable X Server
Allow Docker containers to display GUI applications on your host’s X server.
```bash
xhost +local:docker
```

#### Install NVIDIA Container Toolkit
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

#### Configure Docker to Use the NVIDIA Runtime
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

Restart DockerRestart the Docker service to apply these configuration changes:
```bash
sudo systemctl restart docker
```

**NOTE: IF YOU DON'T HAVE NVIDIA GPU, REMOVE "--runtime=nvidia" FROM .devcontainer.json** 
