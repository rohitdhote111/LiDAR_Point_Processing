# Description
LiDAR point cloud processing for Noise removal and Ground point filtering

This repo and readme file only includes the steps to run the ros packages and Rviz visualization NOT the details about trials and experiments conducted.
Report PDF file is separately provided which includes all the detailed analysis.

# Docker ROS Noetic Workspace Setup

This repository contains instructions on how to set up a Docker environment with ROS Noetic and a Catkin workspace. The Dockerfile included in this repo installs necessary dependencies and creates a Catkin workspace for ROS development.

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/) installed on your machine
- [Visual Studio Code](https://code.visualstudio.com/) installed (optional but recommended)

## Getting Started

### Step 1: Clone or Create a Workspace
```bash
git clone https://github.com/rohitdhote111/TII_Assignment
cd TII_Assignment
```
### Step 2: Create a Docker image using dockerfile

- goto this repo folder 
- Create a docker image using Dockerfile

```bash 
docker  build -t r1_image_from_file .
```
This command will create image with "r1_image_from_file", we can check it inside list of images with command
```
docker images
```
### step 3: Create a Docker container using above image using bash file provided

First give write permission to bash file and then run it

```
chmod +x docker_run.bash
./docker_run.bash
```

### step 4: Copy required files

copy the packages present inside src folder from base machine to container src folder using cp command
Also copy lidar .bag file inside catkin_ws folder of the container
(Run below command outside container)

```
docker cp /path_to_folder/TII_Assignment/src/ cp r13d_container:/root/catkin_ws/
```

### step 5: Perform catkin_make

Goto catkin workspace and perform catkin_make 

```
cd ~/catkin_ws
catkin_make
```

### step 6: Run packages
  Two packages are present inside the src folder
 1. pointcloud_filer
 2. ransac_fast
we can run these packages using roslaunch command
```
roslaunch pointcloud_filer pointcloud_filter.launch
roslaunch ransac_fast ransac_fast.launch
```
### Rviz configuration
After Rviz windows is open select “Fixed Frame>mbuggy/base_link” from left side display panel

Add required pointclouds from Add>By topic>”select point required PC”

Increase point size. (0.03 is good)

Change color of the point cloud if required. (color transform>Flat then select color)
