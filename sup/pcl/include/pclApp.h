/*
 * pclApp.h
 *
 *  Created on: 12.01.2023
 *      Author: joerg
 */
/**
* @file pclApp.h
* @brief Implementation of the class functions to use the PCL library functions.
* @author Jörg Angermayer\n
* @copyright Licensed under MIT
*
* Projekt : kv3 framework\n
*
* @date 09.02.2023 – first implementation
* @date 24.06.2023 – last update
*
*
* @version 1.0
*************************************************************/
#ifndef PCLAPP_H_
#define PCLAPP_H_

#include <iostream>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
using namespace cv;

#include <pcl/point_types.h>
#include <pcl/conversions.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/pcl_macros.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h> //cliping
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h> //noise reduction
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h> //down sample

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "pclUtilitys.h"

using namespace std::chrono_literals;

class pclApp {
public:
	pclApp();
	~pclApp();
	void mainStart();
	void renderLoop();
	bool setupViewerPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr& in);
	bool setupViewerRgbPcl();	
	void updateViewerRgbPclWithRootPclRgb();
	void loadPlyTestPcl();
	void testPclObjectDetectionPipe01();
	void testPclRgbObjectDetectionPipe01();
	void testViewsWithNormals();
	void testViewsWithViewports();
	void testLineCreater(float x, float y, float z);
	void testTranformOrientationToPlane();
	void testBigestPlaneOrientationToPlane();
	void convMatPclToPCL(const cv::Mat &srcPcl, const cv::Mat &srcRgb, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out, int xmax, int ymax);
	void convMatPclToPCL(const cv::Mat &srcPcl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out, int xmax, int ymax);
	void convCvRgbDtoPclRgb(const cv::Mat &srcRGB,const cv::Mat &srcDepth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out);
	void convCvRgbDtoRootPclRgb(const cv::Mat &srcRGB,const cv::Mat &srcDepth);
	void convCvDepthMaptoRootPcl(const cv::Mat &srcDepth, pcl::PointCloud<pcl::PointXYZ>::Ptr& out);

	pclUtilitys *pclUti;
	pcl::PLYReader Reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr rootPcl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rootPclRgb;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rootPclRgbFull;
	pcl::PointCloud<pcl::PointXYZ>::Ptr rootPcl2;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rootPclRgb2;	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rootPclRgb2Full;	
	pcl::PointCloud<pcl::PointXYZ>::Ptr rootPcl3;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rootPclRgb3;		
	pcl::PointCloud<pcl::PointXYZ>::Ptr rootPclResult;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rootPclRgbResult;
	static int rootPclSize;
	static int rootPclRgbSize;
	static int rootPclResultSize;
	static int rootPclRgbResultSize;
};

#endif /* PCLAPP_H_ */
