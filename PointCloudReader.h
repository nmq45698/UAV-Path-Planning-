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
using namespace std;

/// @brief helper function to split a string
/// @details use a delimiter to split the string
/// @param[in, out] str string to be split
/// @param[in, out] regex_str delimiter, like comma, etc.
/// @return a vector of sub strings separated by delimiter
 vector<string> split(const string& str, const string& regex_str);

/// @brief this class is responsible to read the data from .txt files
/// @details this class reads the .txt file, extract x,y,z of each point, then generate 3D scatters
class PointCloudReader
{
public:
	/// @brief constuct an object for point cloud preprocessing
	/// @details the read the format from txt files and pile them up
	/// @param[in] file_path (char*) the file path to the data file
	/// @return a vector m_vTxtPoints saving scattering points
	PointCloudReader(const char* file_path);

	/// @brief generate the cloud from txt data 
	/// @details please give the name of windows and color of point cloud
	/// @return an object for cloud read from txt data
	pcl::PointCloud<pcl::PointXYZ>::Ptr generateFirstCloud();

	/// @brief pop up a console to show the scatters
	/// @details please give the name of windows and color of point cloud
	/// @param[in] cloud used for the cloud to be shown on consoles
	/// @param[in] window_name used for showing consoles
	void showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const string& window_name);

	/// @brief select drone points by rectangle boudary selected
	/// @details give the x,y,z minimum and maximum of drone points
	/// @param[in] input cloud under filtering 
	/// @param[in] x,y,z limits
	pcl::PointCloud<pcl::PointXYZ>::Ptr filterByBoundary(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin,
		const float min, const float max, const string& fn);

private:
	typedef struct tagPOINT_3D
	{
		double x;
		double y;
		double z;
		double r;
	};

	int number_Txt;
	FILE* fp_txt;
	tagPOINT_3D TxtPoint;
	vector<tagPOINT_3D> m_vTxtPoints;
};