#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <fstream>  
#include <string>  
#include <vector>
#include <stdlib.h>
#include <algorithm>
#include <thread>
#include "Meshgrid.h"
using namespace std;

Meshgrid::Meshgrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double& LenOfGrid)
{
	cloudin = cloud;
	pcl::PointXYZ max;
	pcl::PointXYZ min;
	pcl::getMinMax3D(*cloudin, min, max);
	Xmax = max.x; Ymax = max.y; Zmax = max.z;
	Xmin = min.x; Ymin = min.y; Zmin = min.z;
	GridNumX = (int) ((Xmax - Xmin) / LenOfGrid)+1;
	GridNumY = (int) ((Ymax - Ymin) / LenOfGrid)+1;
	GridNumZ = (int) ((Zmax - Zmin) / LenOfGrid)+1;
	GridLength = LenOfGrid;
	NumOverAll = GridNumX * GridNumY * GridNumZ;
	NumOfPoints = cloudin->points.size();
	for (int i = 0; i < cloudin->points.size(); i++)
	{
		tagPOINT_3D pts;
		pts.x = cloudin->points[i].x;
		pts.y = cloudin->points[i].y;
		pts.z = cloudin->points[i].z;
		ptss.push_back(pts);
	}

}

void Meshgrid::gridGenerate()
{
	// Generate the first grid
	/*Grid_3D HeadGrid;
	HeadGrid.Centerpoint.x = Xmin + 0.5 * GridLength;
	HeadGrid.Centerpoint.y = Ymin + 0.5 * GridLength;
	HeadGrid.Centerpoint.z = Zmin + 0.5 * GridLength;
	HeadGrid.dx = 0;
	HeadGrid.dy = 0;
	HeadGrid.dz = 0;*/

	// Generate all grids
	for (int k3 = 0; k3 < GridNumZ; k3++)
	{
		for (int k2 = 0; k2 < GridNumY; k2++)
		{
			for (int k1 = 0; k1 < GridNumX; k1++)
			{
				Grid_3D G;
				G.dx = k1; G.dy = k2; G.dz = k3;
				G.Centerpoint.x = Xmin + GridLength * (k1 + 0.5);
				G.Centerpoint.y = Ymin + GridLength * (k2 + 0.5);
				G.Centerpoint.z = Zmin + GridLength * (k3 + 0.5);
				G.Index = k3 * GridNumY * GridNumX + k2 * GridNumX + k1;
				G.xmin = G.Centerpoint.x - 0.5 * GridLength;
				G.xmax = G.Centerpoint.x + 0.5 * GridLength;
				G.ymin = G.Centerpoint.y - 0.5 * GridLength;
				G.ymax = G.Centerpoint.y + 0.5 * GridLength;
				G.zmin = G.Centerpoint.z - 0.5 * GridLength;
				G.zmax = G.Centerpoint.z + 0.5 * GridLength;
				G.Vacancy = false;
				G.Searched = false;
				Grids.push_back(G);
			}
		}
	}
}

void Meshgrid::connectGrids()
{

}

void Meshgrid::judgeVacancy()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudtemp(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudtemp;
	/*cloudtemp->width = NumOfPoints;
	cloudtemp->height = 1;
	cloudtemp->is_dense = false;
	cloudtemp->points.resize(cloudtemp->width * cloudtemp->height);*/
	for (int k4 = 0; k4 < NumOverAll; k4++)
	{
		/*for (int k5 = 0; k5 < NumOfPoints; k5++)
		{
			cloudtemp->points[k5].x = ptss[k5].x;
			cloudtemp->points[k5].y = ptss[k5].y;
			cloudtemp->points[k5].z = ptss[k5].z;
		}*/

		pcl::copyPointCloud(*cloudin, *cloudtemp);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloudtemp);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(Grids[k4].xmin,Grids[k4].xmax);
		pass.filter(*cloudtemp);

		pass.setInputCloud(cloudtemp);
		pass.setFilterFieldName("y");
		pass.setFilterLimits(Grids[k4].ymin, Grids[k4].ymax);
		pass.filter(*cloudtemp);

		pass.setInputCloud(cloudtemp);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(Grids[k4].zmin, Grids[k4].zmax);
		pass.filter(*cloudtemp);

		if (cloudtemp->points.size() == 0)
		{
			Grids[k4].Vacancy = true;
		}
		
		TrueOrFalse.push_back(Grids[k4].Vacancy);
	}
}

void Meshgrid::drawCubes(const string& window_name)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(window_name));
	viewer->setBackgroundColor(0, 0, 0);
	/*pcl::PointXYZ origin(0, 0, 0);
	pcl::PointXYZ end(1, 3, 2);
	viewer->addLine(origin, end, 255, 0, 0);*/

	/*pcl::PointXYZ origin2(1,3,2);
	pcl::PointXYZ end2(4,5,2);
	viewer->addLine(origin2, end2, 255, 0, 255);*/
	for (int k6 = 0; k6 < NumOverAll; k6++)
	{
		if (TrueOrFalse[k6] == false)
		{
			/*viewer->addCube(
				Grids[k6].xmin,
				Grids[k6].xmax,
				Grids[k6].ymin,
				Grids[k6].ymax,
				Grids[k6].zmin,
				Grids[k6].zmax,
				(rand() % (255 + 1)), (rand() % (255 + 1)), (rand() % (255 + 1)),
				to_string(k6),0
			);*/
			Eigen::Vector3f center(Grids[k6].Centerpoint.x, Grids[k6].Centerpoint.y, Grids[k6].Centerpoint.z);
			Eigen::Quaternionf rotation(0, 0, 0, 0);
			viewer->addCube(center, rotation, GridLength, GridLength, GridLength, to_string(k6), 0);
		}
	}
	//viewer->addCube(0, 1, 0, 1, 0, 1,255,255,0,"a",0);
	//viewer->addCube(1, 2, 1, 2, 1, 2,255,0, 0, "b", 0);
	//while (!viewer->wasStopped())
	//{
	//	this_thread::sleep_for(100ms);
	//}
	viewer->spin();
	system("pause");
}