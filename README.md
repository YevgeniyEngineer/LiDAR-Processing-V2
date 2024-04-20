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
