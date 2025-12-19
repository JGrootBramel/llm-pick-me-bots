# Quickstartguide

1. Prerequisit
2. 

## Prerequisits

### Windows
- Ubuntu-20.04+ on Windows Subsystem for Linux (WSL) (tested with 22.04)
- Docker Desktop
    - enable integration with my default WSL distro

Start WSL on Windows as root user: 
```bash
wsl -u root
```


### Mac

### Ubuntu 20.04 +
- python 3.10
- docker

## Build  the docker Image

From Ubuntu CMD change clone this repository:
 ```bash
 git clone https://github.com/JGrootBramel/llm-pick-me-bots.git
 ```
Change directory and bulid the docker container:
 ```bash
 cd ~/llm-pick-me-bots
 docker build -t noetic-gazebo-rosa -f ./Dockerfile.noetic .
 ```

 If that finishes, start the container:
 ```bash
docker run -it --rm --name limo_sim --net=host -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix noetic-gazebo-rosa
```

## Start the container and get shell
In a  new shell:

```bash
docker exec -it limo_sim bash
```

## 3) Source ROS and the workspace (inside the container)

If not already done in `.bashrc`:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

## 4) Launch the Gazebo simulation

Typical patterns are:

```bash
roslaunch <robot_package> <gazebo_launch>.launch
```
