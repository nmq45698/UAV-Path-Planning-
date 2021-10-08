#pragma once
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <fstream>  
#include <string>  
#include <vector>
#include<stdlib.h>
#include<algorithm>
using namespace std;
class KMeans
{
public:
	typedef struct tagPOINT_3D // cloud points
	{
		double x;
		double y;
		double z;
	};

	typedef struct st_point
	{
		tagPOINT_3D point;
		int groupID;
	};

	typedef struct COLOR_NAME
	{
		double R;
		double G;
		double B;
	};
	/// @brief generate the object for K-means clustering 
	/// @details please give the cloud to be clustered and number of clusters k
	/// @param[in] cloud point cloud to be clustered
	/// @param[in] k number of clusters (max 10)
	/// @param[in] m max iterations limited 
	/// @return an object for cloud read from txt data
	KMeans(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const int & k,const int &m);
	
	/// @brief calculate the Euclidean distance between points 
	/// @details give the points in tagPOINT_3D type
	/// @param[in] pt1 the first point
	/// @param[in] pt2 the second point
	/// @return distance of points
	double distOfPoints(tagPOINT_3D pt1, tagPOINT_3D pt2);

	/// @brief Function for generating random numbers without repeating
	/// @details give the point to be clustered
	/// @param[in] the minimum integer
	/// @param[in] the maximum integer
	vector<int> randperm(int np,int nc);

	/// @brief Tag the points by calculating distance
	/// @details give the st_points.groupID by calculating distance
	void tagOfPoints();

	/// @brief push back points by xyz and groupID
	/// @details transfer xyz and groupID from clouds to at the beginning of the algorithm
	void Initialize();

	bool judgeIterEnd(int k6);

	/// @brief Function for clustering
	/// @details give the point to be clustered
	/// @param[in] cloud point cloud to be clustered
	/// @return an object for cloud read from txt data
	void cluster();

	/// @brief Function for Identification
	/// @details Throw TaggedPoints into different vectors by groupID
	void Identify();

	/// @brief regenerate point cloud and recolor by tags
	/// @details 
	void displayByColor(const string& window_name);

private:
	vector<vector<st_point>> ClassifiedPoints;
	vector<st_point> TaggedPoints; 
	vector<st_point> CenterPoints;
	int NumOfClasses;
	int NumOfPoints;
	int MaxIter;
	int IterGen;
	int R[9] = {255,0,0,160,255,139,32,255,0};
	int G[9] = {0,255,0,32,165,35,178,218,205};
	int B[9] = {0,0,255,240,0,35,170,185,205};
};
