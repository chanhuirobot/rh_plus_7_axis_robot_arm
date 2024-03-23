<div align="center">

![header](https://capsule-render.vercel.app/api?type=venom&color=E9E2D1&height=300&section=header&text=RHplus&fontSize=60&fontColor=30343f)

🔥Tech Stack🔥

<img src="https://img.shields.io/badge/ROS2-22314E?style=flat&logo=ROS&logoColor=white">
<img src="https://img.shields.io/badge/Python-3776AB?style=flat&logo=Python&logoColor=white">
<img src="https://img.shields.io/badge/Notion-90EE90?style=flat&logo=Notion&logoColor=white">
<img src="https://img.shields.io/badge/GitHub-181717?style=flat&logo=GitHub&logoColor=white">
<img src="https://img.shields.io/badge/YouTube-FF0000?style=flat&logo=YouTube&logoColor=white">
</div>

## Building and Running Docker Containers

The process of working with Docker involves two main steps: building Docker images and executing containers.

### 1. Building Docker Images

To build a Docker image, execute the following command:

```bash
sudo docker build -t tr3d:v1.0 .
```

If the process freezes due to memory shortage during the build, consider creating a swap partition using the following commands:
(The swap partition below is temporary and will disappear after rebooting. Also, 16G is a value I set arbitrarily.)
```bash
sudo swapon --show # Check pre-configured swap memory, typically 50% to 65% of total memory.
df -h # Check if there is available space on the hard drive.
sudo fallocate -l 16G /swapfile1 # Create a swap file named swapfile1 with a size of 16G.
sudo chmod 600 /swapfile1
sudo mkswap /swapfile1 # Mark the swap space.
sudo swapon /swapfile1
sudo swapon --show # Confirm the addition of swapfile1.
```
Once the build is complete, you will see a message indicating the build is finished.
<div align="center">

<img src="https://github.com/chanhuirobot/rh_plus_7_axis_robot_arm/assets/81359054/095b230f-277d-4506-b669-633d7084d6f3" width="450" height="300">
</div>

### 2. Running Docker Containers

After building the image, create and run a container using the following command:
```bash
sudo docker run --gpus all --net=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --name tr3d -it tr3d:v1.0 /bin/bash
```

### 3. Restarting Containers
After rebooting, you need to restart the container. Follow these steps:
```bash
sudo docker ps -a # Check Container ID
sudo docker restrat (Container ID)
sudo docker exec -it (Container ID) /bin/bash
```

Also, if you want to exchange files between local and container, you can use the cp command.
```bash
sudo docker cp (from) (to)
# Ex
sudo docker cp ~/tr3d/data/sunrgbd/OFFICIAL_SUNRGBD (Container ID) /tr3d/data/sunrgbd/
```
