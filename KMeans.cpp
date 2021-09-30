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
#include <algorithm>
#include<stdlib.h>
#include"KMeans.h"
using namespace std;

KMeans::KMeans(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const int& k,const int &m)
{
	NumOfPoints = cloud->points.size();
	NumOfClasses = k;
	// Change the pcl clount to st_point (with tags)
	MaxIter = m;
	IterGen = 0;
	for (int kk = 0; kk < NumOfPoints; kk++)
	{
		st_point pts;
		pts.point.x = cloud->points[kk].x;
		pts.point.y = cloud->points[kk].y;
		pts.point.z = cloud->points[kk].z;
		pts.groupID = 2147483647;
		TaggedPoints.push_back(pts);
		/*
		TaggedPoints[kk].point.x = cloud->points[kk].x;
		TaggedPoints[kk].point.y = cloud->points[kk].y;
		TaggedPoints[kk].point.z = cloud->points[kk].z;*/
	}
	TaggedPoints.resize(NumOfPoints);
	CenterPoints.resize(NumOfClasses);
}

vector<int> KMeans::randperm(int np,int nc)
{
	vector<int> temp1;
	vector<int> temp2;
	// temp1 used for save all index possible
	for (int k2 = 0; k2 < np; k2++)
	{
		temp1.push_back(k2+1);
	}
	// rearrange randomly
	random_shuffle(temp1.begin(), temp1.end());
	// temp2 used for 
	for (int k3 = 0; k3 < nc; k3++)
	{
		temp2.push_back(temp1[k3]);
	}
	return temp2;
}

void KMeans::Initialize()
{
	vector<int> temp3(NumOfPoints);
	temp3 = randperm(NumOfPoints,NumOfClasses);
	for (int k4 = 0; k4 < NumOfClasses; k4++)
	{
		// Tag the points selected by groupID
		TaggedPoints[temp3[k4]].groupID = k4;
		// Generate vector points for centers
		
		st_point pts2;
		pts2.point.x = TaggedPoints[temp3[k4]].point.x;
		pts2.point.y = TaggedPoints[temp3[k4]].point.y;
		pts2.point.z = TaggedPoints[temp3[k4]].point.z;
		pts2.groupID = TaggedPoints[temp3[k4]].groupID;
		CenterPoints[k4] = pts2;
	}
	IterGen = 0; 
	cout << "初始化完成" << endl;
}

double KMeans::distOfPoints(tagPOINT_3D pt1, tagPOINT_3D pt2)
{
	double square_distance = pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2);
	double dis = pow(square_distance, 0.5);
	return dis;
}

void KMeans::tagOfPoints()
{
	for (int k3 = 0; k3 < NumOfPoints; k3++)
	{
		st_point pt3 = TaggedPoints[k3];
		// save distance between each center(NumOfClasses)
		vector<double> DistBetCenter;
		DistBetCenter.clear();
		// compare with center point of each classes
		for (int k8 = 0; k8 < NumOfClasses; k8++)
		{
			DistBetCenter.push_back(distOfPoints(pt3.point, CenterPoints[k8].point));
		}
		int PossibleClass = distance(DistBetCenter.begin(), min_element(DistBetCenter.begin(), DistBetCenter.end()));
		TaggedPoints[k3].groupID = PossibleClass;
	}
}

//Caculate Centerpoints according to new groupID
void KMeans::Identify()
{
	ClassifiedPoints.clear();
	for (int k9 = 0; k9 < NumOfClasses; k9++)
	{
		double sumx = 0, sumy = 0, sumz = 0;
		vector<st_point> vm;
		vm.clear();
		for (int k10 = 0; k10 < NumOfPoints; k10++)
		{
			if (TaggedPoints[k10].groupID == k9)
			{
				vm.push_back(TaggedPoints[k10]);
			}
		}

		for (int kk = 0; kk < vm.size(); kk++)
		{
			sumx = sumx + vm[kk].point.x;
			sumy = sumy + vm[kk].point.y;
			sumz = sumz + vm[kk].point.z;
		}

		double x_center = sumx / (vm.size() + 1);
		double y_center = sumy / (vm.size() + 1);
		double z_center = sumz / (vm.size() + 1);

		CenterPoints[k9].point.x = x_center;
		CenterPoints[k9].point.y = y_center;
		CenterPoints[k9].point.z = z_center;

		ClassifiedPoints.push_back(vm);
	}
}

bool KMeans::judgeIterEnd(int k6)
{
	if (k6 < MaxIter)
		return true;
	else
		return false;
}

void KMeans::cluster()
{
	Initialize();
	while (judgeIterEnd(IterGen))
	{
		cout << "正在进行第" << IterGen << "次迭代" << endl;
		tagOfPoints();
		Identify();
		IterGen++;	
	}
}

/*void KMeans::displayByColor(const string& window_name)
{
	//Containers for each classes of clouds
	//vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds(NumOfClasses);
	pcl::PointCloud<pcl::PointXYZ>::Ptr clouds;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer4(new pcl::visualization::PCLVisualizer(window_name));
	int v2 = 0;
	viewer4->createViewPort(0, 0, 1, 1, v2);
	viewer4->setBackgroundColor(255, 255, 255);
	for (int k12 = 0; k12 < NumOfClasses; k12++)
	{
		// Counter for number of clouds of this class
		int idxInnerClass = 0;
		pcl::PointXYZ p = {1,1,1};
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cld = clouds[k12];
		for (int k13 = 0; k13 < NumOfPoints; k13++)
		{
			if (TaggedPoints[k13].groupID == k12)
			{
				idxInnerClass++;
					/*= {
					(float)TaggedPoints[k13].point.x,
					(float)TaggedPoints[k13].point.y,
					(float)TaggedPoints[k13].point.z
				};
				p.x = (float)TaggedPoints[k13].point.x;
				p.y = (float)TaggedPoints[k13].point.y;
				p.z = (float)TaggedPoints[k13].point.z;
				clouds	->push_back(p);
			}
		}	//clouds[k12] = cld;
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sc(clouds,
		KMeans::R[k12],
		KMeans::G[k12],
		KMeans::B[k12]);
		viewer4->addPointCloud(clouds, sc, to_string(1), v2);
	}

	viewer4->addCoordinateSystem(1.0);
	viewer4->spin();
	system("pause");
}*/

void KMeans::displayByColor(const string& window_name)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZRGB>); // 对象：滤波前点云
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer(window_name));
	int v1 = 0;
	clouds->width = NumOfPoints;
	clouds->height = 1;
	clouds->is_dense = false;
	clouds->points.resize(clouds->width * clouds->height);
	
	for (int i = 0; i < NumOfPoints; i++)
	{
		clouds->points[i].x = TaggedPoints[i].point.x;
		clouds->points[i].y = TaggedPoints[i].point.y;
		clouds->points[i].z = TaggedPoints[i].point.z;
		clouds->points[i].r = KMeans::R[TaggedPoints[i].groupID];
		clouds->points[i].g = KMeans::G[TaggedPoints[i].groupID];
		clouds->points[i].b = KMeans::B[TaggedPoints[i].groupID];
	}

	viewer1->createViewPort(0, 0, 1, 1, v1);
	viewer1->setBackgroundColor(255, 255, 255);
	pcl::visualization::PointCloudColorHandlerRGBField <pcl::PointXYZRGB> rgb(clouds);
	viewer1->addPointCloud<pcl::PointXYZRGB>(clouds,"aaa");
	viewer1->addCoordinateSystem(1.0);
	viewer1->spin();
	system("pause");
}