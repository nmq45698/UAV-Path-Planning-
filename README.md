# UAV-Path-Planning with DFS Algorithm
# 基于深度优先搜索算法的无人机路径规划
This is a project using PCL in C++ to plan routes for drones.  
Data from 16-line Lidar are tested and DFS Algorithm are used in route planning.  
It is the first repo uploaded to Github and welcome to pull my project!  
该项目是利用C++中的PCL库，对无人机路径进行规划的一个demo  
采用16线激光雷达数据进行测试，使用深度优先搜索算法进行路线规划  
还请多多关照:blush:

## Introductions
This project takes it origin from an experiment of drones detection.  
With **Optical Camera, Infrared Camera, Millimeterwave Radar and Lidar**  
In this part, we dealt with our **LIDAR** file and there are mainly 3 steps.  
该项目的大背景是基于光学、红外相机、毫米波雷达以及激光雷达数据，对无人机探测和环境感知等探测任务。  
<img src="https://github.com/nmq45698/UAV-Path-Planning-/blob/main/%E7%94%B5%E8%B7%AF%E8%BF%9E%E6%8E%A5.png" width="30%" height="30%">
<img src="https://github.com/nmq45698/UAV-Path-Planning-/blob/main/%E5%AE%9A%E4%BD%8D%E8%AF%AF%E5%B7%AE.png" width="30%" height="30%">
<img src="https://github.com/nmq45698/UAV-Path-Planning-/blob/main/%E5%9B%BE3.png" width="10%" height="10%">  
以下内容为针对激光雷达数据的处理手段 - 环境建模与路径规划

## 1.Data Reading
In this part, we read data from the .txt file of point clouds.  
The format of file is: x  y  z (gapped by tab).  
Each row indicates for one single point.  
See 'PointCloudReader.h' and 'PointCloudReader.cpp'  
该部分内容主要用于读取数据，数据格式需要满足一定规则，按照每行一个点的原则。  
详见 'PointCloudReader.h' and 'PointCloudReader.cpp'

## 2.Environment Modeling

## 3.Route Planning by DFS in C++
