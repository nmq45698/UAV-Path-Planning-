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
#include"Meshgrid.h"
using namespace std;
class PathCalculate
{
public:
	/*typedef struct tagPOINT_3D // cloud points
	{
		double x;
		double y;
		double z;
	};*/

	/*typedef struct Meshgrid::Grid_3D
	{
		tagPOINT_3D Centerpoint; // Co-ordinate of Center Point
		int Index; // Index of grids
		int dx, dy, dz; // Index of grids in x,y,z
		double xmin, xmax, ymin, ymax, zmin, zmax;
		bool Vacancy; // Vacant for 1, barrier for 0
		vector<tagPOINT_3D> PointsInGrid; //Used for save points in cloud
	};*/

	/// @brief initialize path calculation by giving grids
	/// @details give the co-ordinate starting and end points, mesh length
	/// @param[in] X of starting point
	/// @param[in] X of ending point
	/// @param[in] Y of starting point
	/// @param[in] Y of ending point
	/// @param[in] Z of starting point
	/// @param[in] Z of ending point
	/// @param[in] length of cube mesh
	PathCalculate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		const double& xstart, const double& xend,
		const double& ystart, const double& yend,
		const double& zstart, const double& zend, const double len);

	double distOfPoints(Meshgrid::tagPOINT_3D pt1, Meshgrid::tagPOINT_3D pt2);
	
	/// @brief calculate which grid the point is in 
	/// @details input the co-ordinate of xyz and calculate the index of point
	/// @param[in] x of point
	/// @param[in] y of point
	/// @param[in] z of point
	/// @param[out] index of point
	int indexOfBlock(const double& x,const double& y,const double& z);

	/// @brief judge reach the edge according to overall index of grids
	/// @details compare the dx,dy,dz with 0 and NumOfGrids 
	/// @param[in] k index of point
	/// @param[out] reachEnd true if the point 
	bool reachEnd(int k);

	vector<int> neighborhoodFind(int k);

	void DFS(vector<Meshgrid::Grid_3D> grids, const int & IndexOfStart);

	void drawRoutes(const string & window_name);

public:
	vector<Meshgrid::Grid_3D> Grids;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin;
	double Xstart, Xend, Ystart, Yend, Zstart, Zend;
	Meshgrid::tagPOINT_3D StartPoint, EndPoint;
	double Xmin, Xmax, Ymin, Ymax, Zmin, Zmax;
	double LengthOfGrid;
	int GridNumX, GridNumY, GridNumZ;
	vector<Meshgrid::Grid_3D> Routes;
	vector<int> RouteNumber;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudshown;
};