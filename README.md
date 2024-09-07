# TII_Assignment
LiDAR point cloud processing for Noise removal and Ground point filtering

# Docker ROS Noetic Workspace Setup

This repository contains instructions to set up a Docker environment with ROS Noetic and a Catkin workspace. The Dockerfile included in this repo installs necessary dependencies and creates a Catkin workspace for ROS development.

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/) installed on your machine
- [Visual Studio Code](https://code.visualstudio.com/) installed (optional but recommended)

## Getting Started

### Step 1: Clone or Create a Workspace
```bash
git clone https://github.com/rohitdhote111/TII_Assignment
```
### Step 2: Create a Docker image using dockerfile

- goto this repo folder 
- Create a docker image using Dockerfile

```bash 
docker  build -t r1_image_from_file .


