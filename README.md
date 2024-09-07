# TII_Assignment
LiDAR point cloud processing for Noise removal and Ground point filtering

# Docker ROS Noetic Workspace Setup

This repository contains instructions to set up a Docker environment with ROS Noetic and a Catkin workspace. The Dockerfile included in this repo installs necessary dependencies and creates a Catkin workspace for ROS development.

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/) installed on your machine
- [Visual Studio Code](https://code.visualstudio.com/) installed (optional but recommended)

## Getting Started

### Step 1: Clone or Create a Workspace

If you haven't already, create a folder to hold your workspace. Open this folder in Visual Studio Code or any code editor of your choice.

### Step 2: Create a Docker image using dockerfile

Create a docker image using above Dockerfile

Go inside dir using terminal where Dockerfile exists

```bash 
docker  build -t r1_image_from_file .


