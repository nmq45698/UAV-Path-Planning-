#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include<stdlib.h>
#include <fstream>  
#include <string>  
#include <vector>
#include<algorithm>
#include"Meshgrid.h"
#include"PathCalculate.h"
using namespace std;

PathCalculate::PathCalculate(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	const double& xstart, const double& xend,
	const double& ystart, const double& yend,
	const double& zstart, const double& zend, const double len)
{
	Xstart = xstart; Xend = xend;
	Ystart = ystart; Yend = yend;
	Zstart = zstart; Zend = zend;
	Meshgrid msgrd(cloud, len);
	msgrd.gridGenerate();
	msgrd.judgeVacancy();
	Grids = msgrd.Grids;
	Xmin = msgrd.Xmin; Xmax = msgrd.Xmax;
	Ymin = msgrd.Ymin; Ymax = msgrd.Ymax;
	Zmin = msgrd.Zmin; Zmax = msgrd.Zmax;
	GridNumX = msgrd.GridNumX;
	GridNumY = msgrd.GridNumY;
	GridNumZ = msgrd.GridNumZ;
	LengthOfGrid = len;
	StartPoint = { Xstart,Ystart,Zstart };
	EndPoint = { Xend,Yend,Zend };
	cloudshown = cloud;
}

double PathCalculate::distOfPoints(Meshgrid::tagPOINT_3D pt1, Meshgrid::tagPOINT_3D pt2)
{
	double square_distance = pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2);
	double dis = pow(square_distance, 0.5);
	return dis;
}

int PathCalculate::indexOfBlock(const double& x, const double& y, const double& z)
{
	int dx, dy, dz;
	dx = (x - Xmin) / LengthOfGrid;
	dy = (y - Ymin) / LengthOfGrid;
	dz = (z - Zmin) / LengthOfGrid;
	int num = dz * (GridNumX * GridNumY) + dy * (GridNumX)+dx;
	return num;
}

bool PathCalculate::reachEnd(int k)
{
	int dx, dy, dz;
	dx = Grids[k].dx;
	dy = Grids[k].dy;
	dz = Grids[k].dz;
	if ((dx == 0) || (dx = GridNumX))
	{
		return true;
	}
	else if ((dy == 0) || (dy = GridNumY))
	{
		return true;
	}
	else if ((dz == 0) || (dz = GridNumZ))
	{
		return true;
	}
	else
	{
		return false;
	}

}

vector<int> PathCalculate::neighborhoodFind(int k)
{
	vector<int> n; //store the neighborhood grids
	//n.push_back(k); 
	n.push_back(k - 1);
	n.push_back(k + 1);

	n.push_back(k - GridNumX);
	n.push_back(k - GridNumX- 1);
	n.push_back(k - GridNumX + 1);

	n.push_back(k + GridNumX);
	n.push_back(k + GridNumX - 1);
	n.push_back(k + GridNumX + 1);

	n.push_back(k - GridNumX * GridNumY);
	n.push_back(k - GridNumX * GridNumY-1);
	n.push_back(k - GridNumX * GridNumY+1);

	n.push_back(k - GridNumX * GridNumY-GridNumX);
	n.push_back(k - GridNumX * GridNumY - GridNumX - 1);
	n.push_back(k - GridNumX * GridNumY - GridNumX + 1);

	n.push_back(k - GridNumX * GridNumY+GridNumX);
	n.push_back(k - GridNumX * GridNumY + GridNumX - 1);
	n.push_back(k - GridNumX * GridNumY + GridNumX + 1);

	n.push_back(k + GridNumX * GridNumY);
	n.push_back(k + GridNumX * GridNumY-1);
	n.push_back(k + GridNumX * GridNumY+1);

	n.push_back(k + GridNumX * GridNumY-GridNumX);
	n.push_back(k + GridNumX * GridNumY - GridNumX-1);
	n.push_back(k + GridNumX * GridNumY - GridNumX+1);

	n.push_back(k + GridNumX * GridNumY+GridNumX);
	n.push_back(k + GridNumX * GridNumY + GridNumX-1);
	n.push_back(k + GridNumX * GridNumY + GridNumX+1);

	for (int i = 0; i < n.size(); i++)
	{
		if (reachEnd(n[i]) || (Grids[n[i]].Vacancy == false) || (Grids[n[i]].Searched == true))
		{
			n.erase(n.begin() + i);
		}
	}
	return n;
}

void PathCalculate::DFS(vector<Meshgrid::Grid_3D> grids, const int& i)
{
	grids[i].Searched = true; // Set true when stepped into this grid
	vector<int> Neighborhood = neighborhoodFind(i); // Store the index of surrounding points if Vacancy is true
	if ((Neighborhood.size() == 0))
	{
		Routes.pop_back();
		int nxt = RouteNumber.back();
		DFS(grids, nxt);
	}
	else if (i == indexOfBlock(EndPoint.x, EndPoint.y, EndPoint.z))
	{
		cout << "This is the end of path" << endl;
		return;
	}
	else
	{
		Meshgrid::Grid_3D GridPresent = grids[i]; // Grids at present
		cout << "Present Grid Number:" << i << endl;
		Routes.push_back(GridPresent);
		RouteNumber.push_back(i);
		vector<double> Distance(Neighborhood.size());
		// Calculate Distance between neighborhood and destinations
		for (int k = 0; k < Neighborhood.size(); k++)
		{
			Distance[k] = distOfPoints(grids[Neighborhood[k]].Centerpoint, EndPoint);
		}
		// Get the position of minimum value
		vector<double>::iterator min = min_element(begin(Distance), end(Distance));
		int PosOfMin = distance(begin(Distance), min);
		DFS(grids, Neighborhood[PosOfMin]);
		// Select Next Points
	}
}
	
void PathCalculate::drawRoutes(const string & window_name)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudtemp(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudtemp2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloudshown, *cloudtemp);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer(window_name));


	int v1 = 0;
	viewer1->createViewPort(0, 0, 1, 1, v1);
	viewer1->setBackgroundColor(255,255,255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sc(cloudtemp, 0,0,0);
	viewer1->addPointCloud(cloudtemp,sc,to_string(1),v1);
	//pcl::PointXYZ pt = { Xstart,Ystart,Zstart };
	for (int k = 0; k < RouteNumber.size()-1; k++)
	{
		pcl::PointXYZ pt1 = {(float)Routes[k].Centerpoint.x,(float)Routes[k].Centerpoint.y,(float)Routes[k].Centerpoint.z};
		pcl::PointXYZ pt2 = { (float)Routes[k+1].Centerpoint.x,(float)Routes[k+1].Centerpoint.y,(float)Routes[k+1].Centerpoint.z };
		viewer1->addLine(pt1, pt2,255,0,0,to_string(k), v1);
		viewer1->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, to_string(k));
	}
	viewer1->addCoordinateSystem(1.0);
	viewer1->spin();
	system("pause");
}
