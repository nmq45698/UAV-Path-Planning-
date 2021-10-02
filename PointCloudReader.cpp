#include<iostream>
#include"PointCloudReader.h"
#include <regex>

PointCloudReader::PointCloudReader(const char* file_path)
{
	using namespace std;
	fp_txt = fopen(file_path, "r");

	if (fp_txt)
	{
		while (fscanf(fp_txt, "%lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z) != EOF)
		{
			m_vTxtPoints.push_back(TxtPoint);
		}
		number_Txt = m_vTxtPoints.size();
		cout << "数据加载成功" << endl;
	}
	
	else // 未读取成功
	{
		cout << "txt数据加载失败！" << endl;
	}	

}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudReader::generateFirstCloud()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 对象：滤波前点云

	cloud->width = number_Txt;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);
	cout << cloud->width << endl;

	for (int i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = m_vTxtPoints[i].x;
		cloud->points[i].y = m_vTxtPoints[i].y;
		cloud->points[i].z = m_vTxtPoints[i].z;
	}

	return cloud;
}

void PointCloudReader::showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const string& window_name)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer(window_name));
	int v1 = 0;
	viewer1->createViewPort(0, 0, 1, 1, v1);
	viewer1->setBackgroundColor(255, 255, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sc(cloud, 0,0,0);
	viewer1->addPointCloud(cloud, sc, to_string(1), v1);
	viewer1->addCoordinateSystem(1.0);
	viewer1->spin();
	system("pause");
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudReader::filterByBoundary(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin,
	const float min, const float max,const string& fn)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudfiltered(new pcl::PointCloud<pcl::PointXYZ>); 
	pcl::PassThrough<pcl::PointXYZ> pass; 
	pass.setInputCloud(cloudin);
	pass.setFilterFieldName(fn);
	pass.setFilterLimits(min,max);
	pass.setFilterLimitsNegative(false);
	pass.filter(*cloudfiltered);
	return cloudfiltered;
}

std::vector<string> split(const std::string& str, const std::string& regex_str)
{
	std::regex regexz(regex_str);
	std::vector<std::string> list(std::sregex_token_iterator(str.begin(), str.end(), regexz, -1),
		std::sregex_token_iterator());
	return list;
}
