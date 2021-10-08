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
class Meshgrid
{
public:
	// store poins in 3D co-ordinates
	typedef struct tagPOINT_3D
	{
		double x;
		double y;
		double z;
	};
	
	// store grids
	typedef struct Grid_3D
	{
		tagPOINT_3D Centerpoint; // Co-ordinate of Center Point
		int Index; // Index of grids
		int dx, dy, dz; // Index of grids in x,y,z
		double xmin, xmax, ymin, ymax, zmin, zmax;
		bool Vacancy; // Vacant for 1, barrier for 0
		bool Searched;// Visited for 1, else for 0
		vector<tagPOINT_3D> PointsInGrid; //Used for save points in cloud
		// Pointer for neighborhoods
		/*Grid_3D* up;		
		Grid_3D* up_n;		
		Grid_3D* up_w;
		Grid_3D* up_e;
		Grid_3D* up_s;
		Grid_3D* up_nw;
		Grid_3D* up_ne;
		Grid_3D* up_se;
		Grid_3D* up_sw;
		Grid_3D* down;
		Grid_3D* down_n;
		Grid_3D* down_w;
		Grid_3D* down_e;
		Grid_3D* down_s;
		Grid_3D* down_nw;
		Grid_3D* down_ne;
		Grid_3D* down_se;
		Grid_3D* down_sw;
		Grid_3D* row_n;
		Grid_3D* row_w;
		Grid_3D* row_e;
		Grid_3D* row_s;
		Grid_3D* row_nw;
		Grid_3D* row_ne;
		Grid_3D* row_se;
		Grid_3D* row_sw;*/
	};

	/// @brief generate cube grids for point clouds
	/// @details please give the point cloud / length of cube in a grid 
	/// @param[in] cloud point cloud to be clustered
	/// @param[in] length of grids to be 
	/// @return an object for cloud read from txt data
	Meshgrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double& LenOfGrid);
	
	/// @brief generate grids
	/// @details give the new grid and its center point axis
	void gridGenerate();

	/// @brief make connections between neighborhood grids
	/// @details fill the pointers part of grids
	void connectGrids();
	
	/// @brief judge if there are points in the grid
	/// @details 
	void judgeVacancy();

	/// @brief draw the cube that is not vacant
	void drawCubes(const string& window_name);

public:
	double Xmax, Xmin;
	double Ymax, Ymin;
	double Zmax, Zmin;
	int GridNumX, GridNumY, GridNumZ;
	double GridLength;
	int NumOverAll;
	int NumOfPoints;
	vector<Grid_3D> Grids;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin;
	
	vector<bool>	TrueOrFalse;
	vector<tagPOINT_3D> ptss;
};