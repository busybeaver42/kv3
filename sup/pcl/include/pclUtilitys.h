/*
 * pclUtilitys.h
 *
 *  Created on: 14.01.2023
 *      Author: joerg
 */
/**
* @file pclUtilitys.h
* @brief Implementation of the class functions to support the user with usefull functions.
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
#ifndef PCLUTILITYS_H_
#define PCLUTILITYS_H_

#include <iostream>
#include <thread>
#include <boost/thread/thread.hpp>
#include <vector>
#include <ctime>
#include <Eigen/Core>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/common/common.h> // for getMinMax3D
#include <pcl/common/transforms.h>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/pcl_macros.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h> //cliping
#include <pcl/filters/voxel_grid.h> //down sample
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h> //noise reduction
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h> 
#include <pcl/range_image/range_image.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h> //pre object detection
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h> 
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>

//MESH
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>


typedef pcl::PointWithScale PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZINormal PointTypeFull;

typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::FPFHSignature33 LocalDescriptorT;
typedef pcl::PointCloud<LocalDescriptorT>::Ptr LocalDescriptorsPtr;

typedef pcl::PointXYZ ICPPointT;
typedef pcl::PointCloud<ICPPointT> ICPPointCloud;
typedef pcl::PointCloud<ICPPointT>::Ptr ICPPointCloudPtr;


using namespace std;
using namespace pcl;

class pclUtilitys {
public:
	pclUtilitys();
	~pclUtilitys();
	void createWallPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out);
	void setWallrelativeToOriginXY(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out);
	void getBiggestPlaneLocation(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointXYZ& out);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr createLineToZaxis(int length, float stepSize);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr createAxis(float x, float y, float z);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr createAxis(float x, float y, float z, int length, float stepSize);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr createDINA4PaperContour();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadPCL(const string pathToPcd);
	void convertZPcl(pcl::PointCloud<pcl::PointXYZRGB>& in);
	void convertCordinateSystemFromOpenCvToPlcViewer(const double &inx, const double &iny, const double &inz, double &outx, double &outy, double &outz);
	void convertCordinateSystemFromOpenCvToPlcViewer(const float &inx, const float &iny, const float &inz, float &outx, float &outy, float &outz);
	void setPclRGBColor(pcl::PointCloud<pcl::PointXYZRGB>& in, float r, float g, float b);
	void mergeXYZRGBPcl(const pcl::PointCloud<pcl::PointXYZRGB>& in1, const pcl::PointCloud<pcl::PointXYZRGB>& in2, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out);
	void mergeXYZPcl(const pcl::PointCloud<pcl::PointXYZ>& in1, const pcl::PointCloud<pcl::PointXYZ>& in2, pcl::PointCloud<pcl::PointXYZ>::Ptr& out);
	void convertXYZBGRtoRGB(pcl::PointCloud<pcl::PointXYZRGB>& in);
	void convertXYZRGBtoXYZ(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZ>::Ptr out);
	void convertPtrXYZtoXYZRGB(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out);
	void convertPtrXYZRGBtoXYZ(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZ>::Ptr& out);
	void convertXYZRGBtoXYZI(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZINormal>::Ptr out);
	void convertXYZItoXYZRGB(const pcl::PointCloud<pcl::PointXYZINormal>& in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out);
	void convertXYZRGBtoXYZRGBNormal(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr out);
	void convertXYZRGBNormaltoXYZRGB(const pcl::PointCloud<pcl::PointXYZRGBNormal>& in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out);
	static bool customRegionGrowing(const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance);
	void getNormalsFromCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointNormal>& normals);
	void getNormalsFromCloudRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::Normal>& normals);
	void getPointNormalsFromCloudRgb(const pcl::PointCloud<pcl::PointXYZRGB>& input, pcl::PointCloud<pcl::PointNormal>& cloud_with_normals);
	void getSmoothPointNormalsFromCloudRgb(const pcl::PointCloud<pcl::PointXYZRGB>& input, pcl::PointCloud<pcl::PointNormal>& cloud_with_normals);
	void getNormalsXYZIFromCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZINormal>& cloudOut);
	void getNormalsXYZIFromCloudRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZINormal>& cloudOut);
	void extractPlaneFromCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& cloudOut, bool bSegPlane);
	void extractPlaneFromCloudRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut, bool bSegPlane);
	void voxelGrid(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& cloudOut, float distanceThreshold);
	void voxelGridRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut, float distanceThreshold);
	void passTroughFilter(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& cloudOut, float minfl, float maxfl);
	void passTroughFilterRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut, float minfl, float maxfl);
	void cropFilterRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut, float minX, float maxX, float minY, float maxY);
	void statisticalOutlierRemovalFilter(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& cloudOut, int numbersOfNeighbors, float deviationMul);
	void statisticalOutlierRemovalFilterRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut, int numbersOfNeighbors, float deviationMul);
	//void radiusOutlierRemovalFilter(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut, int mode, float radius, int numbersOfNeighbors);
	//void radiusOutlierRemovalFilter(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut);
	void radiusOutlierRemovalhelpFilter(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointXYZ>& out, int mode, float radius, int numbersOfNeighbors);
	void doConditionalEuclideanClusteringRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut);
	void doEuclideanClustering(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& out);
	void doEuclideanClusteringRgb(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out);
	void testDoEuclideanClusteringRgb(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out);
	void movingLeastSquaresFilerRgb(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut, int polyOrder, float radius);
	void createRangeImageFromPcl(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::RangeImage& rangeImage);
	void doRegionGrowingSegmentationColor(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out);
	void doRegionGrowingSegmentation(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointXYZRGB>& out);
	void doCylinderSegmentationRgb(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out);
	void momentOfInertiaTest(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointXYZ>& out);
	void momentOfInertiaTestRgb(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out);
	void momentOfInertiaRgb(const pcl::PointCloud<pcl::PointXYZRGB>& in);
	void testRandomSampleConsensus();
	void narfKeypointExtraction(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& cloudOut);
	void narfKeypointExtractionRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut);
	void icpSimple(const pcl::PointCloud<pcl::PointXYZRGB>& in1, const pcl::PointCloud<pcl::PointXYZRGB>& in2, pcl::PointCloud<pcl::PointXYZRGB>& out);
	void icpNormals(const pcl::PointCloud<pcl::PointXYZRGB>& in1, const pcl::PointCloud<pcl::PointXYZRGB>& in2, pcl::PointCloud<pcl::PointXYZRGB>& out);
	void rotateX(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut, float radiantAngle);
	void rotateY(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut, float radiantAngle);
	void rotateZ(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut, float radiantAngle);
	void rotateEulerXYZ(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut,const float xAngle, const float yAngle,const float zAngle);
	void rotateEulerYZX(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut,const float yAngle, const float zAngle,const float xAngle);
	void translationXYZ(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out, float x, float y, float z);
	void keypointExtractionNARF(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointXYZ>& out);
	void keypointExtractionNARF(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointWithScale>& out);
	void keypointExtractionISS(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out);
	void keypointExtractionSIFT(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out);
	void keypointExtractionSIFT(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointWithScale>& out);
	void keypointExtractionSIFT(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointNormal>::Ptr& normalCloud, pcl::PointCloud<pcl::PointWithScale>& out);
	void keypointExtractionHarris(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out);
	void FastPointFeatureHistogramsExample(const pcl::PointCloud<pcl::PointXYZRGB>& in1, const pcl::PointCloud<pcl::PointXYZRGB>& in2, pcl::PointCloud<pcl::PointXYZRGB>& out);
	void extractFPFHfromKeypoints(const pcl::PointCloud<pcl::PointXYZ>& in1, const pcl::PointCloud<pcl::PointXYZ>& in2,
                                           const pcl::PointCloud<pcl::PointWithScale>& inKP1, const pcl::PointCloud<pcl::PointWithScale>& inKP2,
										   const pcl::PointCloud<pcl::PointNormal>& inN1, const pcl::PointCloud<pcl::PointNormal>& inN2,
										    pcl::PointCloud<pcl::PointXYZ>& out);
	void extractFPFHfromKeypoints(const pcl::PointCloud<pcl::PointXYZ>& in1, const pcl::PointCloud<pcl::PointXYZ>& in2,
                                           const pcl::PointCloud<pcl::PointWithScale>& inKP1, const pcl::PointCloud<pcl::PointWithScale>& inKP2,
										   const pcl::PointCloud<pcl::PointNormal>& inN1, const pcl::PointCloud<pcl::PointNormal>& inN2,
										    Eigen::Matrix4f& tform);
	void reconTwoPcl(const pcl::PointCloud<pcl::PointXYZ>& in1, const pcl::PointCloud<pcl::PointXYZ>& in2, Eigen::Matrix4f& tform);

	//Viewer
	void ViewSimplePcl(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	//pcl::visualization::PCLVisualizer::Ptr ViewSimplePcl (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);	
	//pcl::visualization::PCLVisualizer::Ptr viewRgb (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	void viewRgb(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	pcl::visualization::PCLVisualizer::Ptr viewRgbDouble (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud2);
	pcl::visualization::PCLVisualizer::Ptr viewRgbNormals (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals);
	pcl::visualization::PCLVisualizer::Ptr viewportsVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRgb);
	pcl::visualization::PCLVisualizer::Ptr viewportsDiffVisRgb (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRgb);
	pcl::visualization::PCLVisualizer::Ptr viewportsDiffVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2);
	//pcl::visualization::PCLVisualizer::Ptr simpleRangeImgVis(pcl::RangeImage::Ptr& range_image_ptr);
	//pcl::visualization::PCLVisualizer::Ptr simpleXZIVis (pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud);

	void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);

	pcl::visualization::PCLVisualizer::Ptr viewer;
	static int rootPclSize;
	static int rootPclRgbSize;
	static int rootPclResultSize;
	static int rootPclRgbResultSize;
	static pcl::RangeImage& range_image;
	static pcl::visualization::RangeImageVisualizer range_image_widget;
	Eigen::Matrix4f transM;
	//pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	static pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_xyz;// normal search methode
	static pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_xyzrgb;// normal search methode

	private:
	Eigen::Matrix4f computeInitialAlignment (const PointCloudPtr & source_points, const LocalDescriptorsPtr & source_descriptors,
                         const PointCloudPtr & target_points, const LocalDescriptorsPtr & target_descriptors,
                         float min_sample_distance, float max_correspondence_distance, int nr_iterations);
	Eigen::Matrix4f refineAlignment (const ICPPointCloudPtr & source_points, const ICPPointCloudPtr & target_points,
                 const Eigen::Matrix4f initial_alignment, float max_correspondence_distance,
                 float outlier_rejection_threshold, float transformation_epsilon, float max_iterations);						 
	//void DetectSIFTKeypoints(PointCloud<PointXYZRGB>::Ptr &points, PointCloud<PointWithScale>::Ptr &keypoints);
	void DetectSIFTKeypoints(PointCloud<PointXYZRGB>::Ptr &points, PointCloud<PointT>::Ptr &keypoints);
	void DetectHarrisKeypoints(PointCloud<PointXYZRGB>::Ptr &points, PointCloud<PointXYZI>::Ptr &keypoints);
	void DetectISSKeypoints(PointCloud<PointXYZRGB>::Ptr &points, PointCloud<PointXYZRGB>::Ptr &keypoints);
	void DetectNARFKeypoints(PointCloud<PointXYZ>::Ptr &points, PointCloud<PointXYZ>::Ptr &keypoints);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr compute_fpfh_feature(pcl::PointCloud<pcl::PointXYZ> ::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree);

	public:
	void meshingViaFastTriangulation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, std::string filePath, int pointNormalMode);
	void meshingViaFastTriangulation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, pcl::PolygonMesh &triangles, int pointNormalMode);
	void meshingViaPoisson(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, std::string filePath, int pointNormalMode);
	void meshingViaPoisson(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, pcl::PolygonMesh &triangles, int pointNormalMode);
	void PointCloud2Vector3d (pcl::PointCloud<PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data);
	void meshingBsplineFitting(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, pcl::PolygonMesh &triangles);

	
	
};

#endif /* PCLUTILITYS_H_ */
