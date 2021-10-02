# UAV-Path-Planning with DFS Algorithm
# 基于深度优先搜索算法的无人机路径规划
This is a project using PCL in C++ to plan routes for drones.  
Data from 16-line Lidar are tested and DFS Algorithm are used in route planning.  
It is the first repo uploaded to Github and welcome to pull my project!  
该项目是利用C++中的PCL库，对无人机路径进行规划的一个demo  
采用16线激光雷达数据进行测试，使用深度优先搜索算法进行路线规划  
第一个Repo,不足之处还请多多指正:blush:  
<img src="https://github.com/nmq45698/UAV-Path-Planning-/blob/main/%E6%A8%A1%E5%9E%8B%E7%AE%80%E5%9B%BE.jpg" width="10%" height="10%">

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
In order to deal with huge number of data - 1MB per 10ms, which depends on the accuracy and sample rate of lidar,  
we sample the point cloud into cube models by judging whether there are points in a cube,  
considering the problem of huge amount of data, bandwidth limit for communication, power restriction and safety.
See 'Meshgrid.h' and 'Meshgrid.cpp'  
该部分内容主要用于建立网格模型，将有点云的点所在的方格视为障碍物方格。 
主要考虑大数据量下，简化的模型可以有限传输带宽留给通信系统，并且节省因为数据量大导致的功率问题，以及飞行安全  
<img src="https://github.com/nmq45698/UAV-Path-Planning-/blob/main/%E6%A8%A1%E5%9E%8B%E7%AE%80%E5%9B%BE.jpg" width="20%" height="20%">
## 3.Route Planning by DFS in C++  
Core part of the project and in this part we use DFS algorithm, which will either return or go in depth, depending on its neighborhood.
**Whether it could go, where to go, and go on!**
See 'PathCalculate.h' and 'PathCalculate.cpp'  
算法的关键部分。深度优先搜索 - 不进则退。
【能不能进，往哪进，进行】无限循环！
