/*
 * pclApp.cpp
 *
 *  Created on: 12.01.2023
 *      Author: joerg
 */
/**
 * @file  pclApp.cpp
 * @brief Implementation of the class functions to use the PCL library functions.
 * @brief created on:  09.02.2023;  last update: 24.06.2023
 * @author Jörg Angermayer\n
 * @copyright Licensed under MIT
 */
#ifdef USE_PCL
#include "pclApp.h"


int pclApp::rootPclSize = 0;
int pclApp::rootPclRgbSize = 0;
int pclApp::rootPclResultSize = 0;
int pclApp::rootPclRgbResultSize = 0;

pclApp::pclApp() {
		//pclUtilitys myPclUtilitysObj = pclUtilitys();
		//pclApp::pclUti = &myPclUtilitysObj;		
}
pclApp::~pclApp() {}

 /**
  * convert opencv Mat PCL image and opencv color mat image to pointcloud library color PCL
  * input:\n
  * \param	Mat PCL, Mat colorImage, int xmax = pcl width, int ymax pcl height
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out
  * \return	void
  *
  */
void pclApp::convMatPclToPCL(const cv::Mat &srcPcl, const cv::Mat &srcRgb, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out, int xmax, int ymax){
	long int counter = 0;
    int y=0;
	int x=0;
    y=0;
    while(y<ymax){
    	x=0;
		while(x<xmax){			
			float z = srcPcl.at<cv::Vec3f>(y,x)[2];//pcl z value			
			if((z > 0.001)||(z < -0.001)){
				pcl::PointXYZRGB point;
				point.x = srcPcl.at<cv::Vec3f>(y,x)[0];
				point.y = srcPcl.at<cv::Vec3f>(y,x)[1];
				point.z = srcPcl.at<cv::Vec3f>(y,x)[2];
				point.r = srcRgb.at<Vec3b>(y,x)[2];
				point.g = srcRgb.at<Vec3b>(y,x)[1];
				point.b = srcRgb.at<Vec3b>(y,x)[0];
				out->push_back(point);
				counter++;
			}
			x++;
		}
		y++;
    }	
	//cout << "counter: " << counter << endl;
	out->points.resize((uint32_t)counter);
}

 /**
  * convert opencv Mat PCL image to pointcloud library color PCL
  * input:\n
  * \param	Mat PCL, int xmax = pcl width, int ymax pcl height
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out
  * \return	void
  *
  */
void pclApp::convMatPclToPCL(const cv::Mat &srcPcl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out, int xmax, int ymax){
	long int counter = 0;
    int y=0;
	int x=0;
    y=0;
    while(y<ymax){
    	x=0;
		while(x<xmax){			
			float z = srcPcl.at<cv::Vec3f>(y,x)[2];//pcl z value			
			if((z > 0.001)||(z < -0.001)){
				pcl::PointXYZRGB point;
				point.x = srcPcl.at<cv::Vec3f>(y,x)[0];
				point.y = srcPcl.at<cv::Vec3f>(y,x)[1];
				point.z = srcPcl.at<cv::Vec3f>(y,x)[2];
				out->push_back(point);
				counter++;
			}
			x++;
		}
		y++;
    }	
	//cout << "counter: " << counter << endl;
	out->points.resize((uint32_t)counter);
}

 /**
  * convert opencv RGB image and opencv depth image to pointcloud library color PCL
  * input:\n
  * \param	Mat RGB image, depth image
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out
  * \return	void
  *
  */
void pclApp::convCvRgbDtoPclRgb(const cv::Mat &srcRGB,const cv::Mat &srcDepth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out){
	long int counter = 0;
	const float factor = 1;//0.001;	
    int y=0;
	int x=0;
	int ymax = srcRGB.size().height;
	int xmax = srcRGB.size().width;
    y=0;
    while(y<ymax){
    	x=0;
		while(x<xmax){				
			float z = (float)((srcDepth.at<ushort>(y,x)) * factor);	
			if((z > 0.001)||(z < -0.001)){
				pcl::PointXYZRGB point;
				point.z = (float)z;
				point.x = (float)x*factor;
				point.y = (float)y*factor;
				point.r = (uint8_t)srcRGB.at<cv::Vec3b>(y,x)[0];
				point.g = (uint8_t)srcRGB.at<cv::Vec3b>(y,x)[1];
				point.b = (uint8_t)srcRGB.at<cv::Vec3b>(y,x)[2];
				out->push_back(point);
				counter++;
			}
			x++;
		}
		y++;
    }	
	out->points.resize((uint32_t)counter);
}

 /**
  * convert opencv RGB image and opencv depth image to pointcloud library color PCL
  * input:\n
  * \param	Mat RGB image, depth image
  * output:\n
  * \param	private rootPclRgb
  * \return	void
  *
  */
void pclApp::convCvRgbDtoRootPclRgb(const cv::Mat &srcRGB, const cv::Mat &srcDepth){
	if(rootPclRgb != NULL){
		rootPclRgb->clear();
	}else{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr myCapturedPcl (new pcl::PointCloud<pcl::PointXYZRGB>);
		rootPclRgb = myCapturedPcl;
	}
	long int counter = 0;
	const float factor = 0.001;	
    int y=0;
	int x=0;
	int ymax = srcRGB.size().height;
	int xmax = srcRGB.size().width;
    y=0;
    while(y<ymax){
    	x=0;
		while(x<xmax){			
			float z = (float)((srcDepth.at<ushort>(y,x)) * factor);		
			if((z > 0.001)||(z < -0.001)){
				pcl::PointXYZRGB point;
				point.x = z;
				point.y = (float)x*factor;
				point.z = (float)y*factor;
				point.r = srcRGB.at<cv::Vec3b>(y,x)[2];
				point.g = srcRGB.at<cv::Vec3b>(y,x)[1];
				point.b = srcRGB.at<cv::Vec3b>(y,x)[0];
				rootPclRgb->push_back(point);
				counter++;
			}
			x++;
		}
		y++;
    }	
	rootPclRgb->points.resize((uint32_t)counter);	
}

 /**
  * convert opencv DepthMap to pointcloud
  * input:\n
  * \param	Mat depth image
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZ>::Ptr
  * \return	void
  *
  */
void pclApp::convCvDepthMaptoRootPcl(const cv::Mat &srcDepth, pcl::PointCloud<pcl::PointXYZ>::Ptr& out){
	long int counter = 0;
	const float factor = 0.001;	
    int y=0;
	int x=0;
	int ymax = srcDepth.size().height;
	int xmax = srcDepth.size().width;
    y=0;
    while(y<ymax){
    	x=0;
		while(x<xmax){			
			float z = (float)((srcDepth.at<ushort>(y,x)) * factor);		
			if((z > 0.001)||(z < -0.001)){
				pcl::PointXYZ point;
				point.x = z;
				point.y = (float)x*factor;
				point.z = (float)y*factor;
				out->push_back(point);
				counter++;
			}
			x++;
		}
		y++;
    }	
	out->points.resize((uint32_t)counter);
}



 /**
  * load ply to different formats
  * \return	void
  *
  */
void pclApp::loadPlyTestPcl(){
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl01 (new pcl::PointCloud<pcl::PointXYZ>);
	rootPcl = pcl01;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl02 (new pcl::PointCloud<pcl::PointXYZRGB>);
	rootPclRgb = pcl02;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl03 (new pcl::PointCloud<pcl::PointXYZ>);
	rootPcl2 = pcl03;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl04 (new pcl::PointCloud<pcl::PointXYZRGB>);
	rootPclRgb2 = pcl04;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl05 (new pcl::PointCloud<pcl::PointXYZ>);
	rootPcl3 = pcl05;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl06 (new pcl::PointCloud<pcl::PointXYZRGB>);
	rootPclRgb3 = pcl06;	

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl07 (new pcl::PointCloud<pcl::PointXYZRGB>);
	rootPclRgbFull = pcl07;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl08 (new pcl::PointCloud<pcl::PointXYZRGB>);
	rootPclRgb2Full = pcl08;	
/*
	Reader.read("/media/hd2/db/RGBD/kv2/kv2Rec/5_042190541047_PCL.ply", *rootPcl);
	Reader.read("/media/hd2/db/RGBD/kv2/kv2Rec/5_042190541047_PCLrgb.ply", *rootPclRgb);	//0_042190541047PCLEdgeRgb.ply //0_042190541047_PCLrgb
	Reader.read("/media/hd2/db/RGBD/kv2/kv2Rec/6_042190541047_PCL.ply", *rootPcl2);
	Reader.read("/media/hd2/db/RGBD/kv2/kv2Rec/6_042190541047_PCLrgb.ply", *rootPclRgb2);
	Reader.read("/media/hd2/db/RGBD/kv2/kv2Rec/7_042190541047_PCL.ply", *rootPcl3);
	Reader.read("/media/hd2/db/RGBD/kv2/kv2Rec/7_042190541047_PCLrgb.ply", *rootPclRgb3);	
*/
	//Reader.read("/media/hd2/db/RGBD/kv2/kv2Rec/5_042190541047_PCL.ply", *rootPcl);
	Reader.read("/media/hd2/db/RGBD/kv2/kv2Rec/5_042190541047PCLEdgeRgb.ply", *rootPclRgb);	//0_042190541047PCLEdgeRgb.ply //0_042190541047_PCLrgb
	//Reader.read("/media/hd2/db/RGBD/kv2/kv2Rec/6_042190541047_PCL.ply", *rootPcl2);
	Reader.read("/media/hd2/db/RGBD/kv2/kv2Rec/6_042190541047PCLEdgeRgb.ply", *rootPclRgb2);
	//Reader.read("/media/hd2/db/RGBD/kv2/kv2Rec/7_042190541047_PCL.ply", *rootPcl3);
	Reader.read("/media/hd2/db/RGBD/kv2/kv2Rec/7_042190541047PCLEdgeRgb.ply", *rootPclRgb3);	

	Reader.read("/media/hd2/db/RGBD/kv2/kv2Rec/5_042190541047_PCLrgb.ply", *rootPclRgbFull);
	Reader.read("/media/hd2/db/RGBD/kv2/kv2Rec/6_042190541047_PCLrgb.ply", *rootPclRgb2Full);

	//pclUti->convertXYZBGRtoRGB(*rootPclRgb); PCLEdgeRgb
	pclUti->convertXYZRGBtoXYZ(*rootPclRgb, rootPcl);
	pclUti->convertXYZRGBtoXYZ(*rootPclRgb2, rootPcl2);
	pclUti->convertXYZRGBtoXYZ(*rootPclRgb3, rootPcl3);
	pclApp::rootPclSize = (int)rootPcl->size();
	pclApp::rootPclRgbSize = (int)rootPclRgb->size();
}

 /**
  * Render loop
  * \return	void
  *
  */
void pclApp::renderLoop(){
	/*
	float angular_resolution_x = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
	float angular_resolution_y = (float) (  1.0f * (M_PI/90.0f));  //   1.0 degree in radians
	float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
	float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
	float noiseLevel=0.00;
	float minRange = 0.0f;
	int borderSize = 1;
    */
	while (!pclUti->viewer->wasStopped ()){
		//pclUti->range_image_widget.spinOnce ();
		pclUti->viewer->spinOnce (1);
		std::this_thread::sleep_for(33ms);
		/*
		scene_sensor_pose = pclUti->viewer->getViewerPose();
		pclUti->range_image->createFromPointCloud (*rootPcl, angular_resolution_x, angular_resolution_y,
										  pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
										  scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noiseLevel, minRange, borderSize);
		pclUti->range_image_widget.showRangeImage (*pclUti->range_image);
		*/
	}
}


 /**
  * view the normals
  * \return	void
  *
  */
void pclApp::testViewsWithNormals(){
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> openCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr myCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr myNormals (new pcl::PointCloud<pcl::Normal>);

	pclUti->voxelGridRgb(*rootPclRgb, *rootPclRgb, 0.06f);
	pclUti->statisticalOutlierRemovalFilterRgb(*rootPclRgb, *rootPclRgb, 50, 1.0);

	pclUti->getNormalsFromCloudRgb(*rootPclRgb, *myNormals);
	pclUti->viewer = pclUti->viewRgbNormals(rootPclRgb, myNormals);

	renderLoop();
}

 /**
  * view with viewports
  * \return	void
  *
  */
void pclApp::testViewsWithViewports(){
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudVRgb (new pcl::PointCloud<pcl::PointXYZRGB>);

	//viewer = viewportsVis(rootPcl,rootPclRgb);
	//pclUti->voxelGridRgb(*rootPclRgb, *cloudVRgb, 0.04f);
	//pclUti->viewer = pclUti->viewportsDiffVisRgb(rootPclRgb, cloudVRgb);
	//////////////////////////pclUti->viewer = pclUti->viewRgb(rootPclRgb);
	pclUti->viewRgb(rootPclRgb);
	renderLoop();
}

 /**
  * view Rgb pointcloud - setup viewer
  * \return	bool  -  1 = the setup was succesfull
  *
  */
bool pclApp::setupViewerPcl(pcl::PointCloud<pcl::PointXYZ>::Ptr& in){
	bool retval = false;

	if(in != NULL){
		retval = true;
		pclApp::pclUti->ViewSimplePcl(in);
		//cout << "Size: " << in->size() << endl;
	}else{
		cout << "NULL - setupViewerPcl" << endl;
		retval = false;
	}
	
	return(retval);
}

 /**
  * view Rgb pointcloud - setup viewer
  * \return	bool  -  1 = the setup was succesfull
  *
  */
bool pclApp::setupViewerRgbPcl(){
	bool retval = false;
	//pclUti->viewRgb(rootPclRgb);
	pclApp::pclUti->viewer->setBackgroundColor(140, 0, 0);
	pclApp::pclUti->viewer->addCoordinateSystem (1.0);
    pclApp::pclUti->viewer->initCameraParameters ();

	if(rootPclRgb != NULL){
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(rootPclRgb);
		retval = true;
	}else{
		cout << "NULL" << endl;
		retval = false;
	}
	return(retval);
}

 /**
  * view Rgb pointcloud - update viewer via rootPclRgb
  * \return	void
  *
  */
void pclApp::updateViewerRgbPclWithRootPclRgb(){
	if(rootPclRgb != NULL){
		if(pclUti != NULL){
			if(pclUti->viewer != NULL){
				pclUti->viewer->removeAllPointClouds();
			}
		}
	}else{
		cout << "NULL" << endl;
	}
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(rootPclRgb);
  	//pclUti->viewer->addPointCloud<pcl::PointXYZRGB> (rootPclRgb, rgb, "sample cloud");
}

 /**
  * test to detect object inside RGB PCL
  * \return	void
  *
  */
void pclApp::testPclRgbObjectDetectionPipe01(){
	//////////////////////////////////////////
	//PCL RGB
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudBufferRgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudClusterRgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudMergedRgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDummyOutRgb (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIOut (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclfilter1 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclfilter2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBuffer (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMerged (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZCluster (new pcl::PointCloud<pcl::PointXYZ>);
	//////////////////////////////////////////
	// Rotate
	float theta;
	//theta = (float)((float)180.0f*(float)((float)M_PI)/(float)180.0f);
	//cout << "theta:" << theta << endl;
	//pclUti->rotateZ(*rootPclRgb, *cloudBufferRgb, theta);
	//////////////////////////////////////////
	//  Prepocess
	//pclUti->voxelGridRgb(*rootPclRgb, *cloudBufferRgb, 0.02f); //do not use before keypoint detection
	//pclUti->statisticalOutlierRemovalFilterRgb(*rootPclRgb, *cloudBufferRgb, 300, 1.0); //ok


	/////////////////////
	// TEST AREA
	/*
	pcl::RangeImage::Ptr range_image_New(new pcl::RangeImage);
	pclUti->createRangeImageFromPcl(*rootPcl, rangeImage); //ok
	*range_image_New = rangeImage;
	*/
	/////
//	pclUti->voxelGrid(*rootPcl, *cloudBuffer, 0.02f); //ok
//	pclUti->statisticalOutlierRemovalFilter(*cloudBuffer, *cloudBuffer, 300, 1.5); //ok
	/*
	pclUti->extractPlaneFromCloud(*cloudBuffer, *cloudBuffer, true);//ok
	pclUti->extractPlaneFromCloud(*cloudBuffer, *cloudBuffer, true);//ok
	pclUti->extractPlaneFromCloud(*cloudBuffer, *cloudBuffer, true);//ok
	pclUti->extractPlaneFromCloud(*cloudBuffer, *cloudBuffer, true);//ok
	pclUti->extractPlaneFromCloud(*cloudBuffer, *cloudBuffer, true);//ok
	pclUti->doEuclideanClustering(*cloudBuffer, *cloudXYZCluster);
	*/

	//pclUti->voxelGridRgb(*rootPclRgb, *cloudBufferRgb, 0.02f); //ok 0.02f is nice compromise
	//pclUti->statisticalOutlierRemovalFilterRgb(*cloudBufferRgb, *cloudBufferRgb, 300, 1.5); //ok
	/*
	pclUti->extractPlaneFromCloudRgb(*cloudBufferRgb, *cloudBufferRgb, true);//ok
	pclUti->extractPlaneFromCloudRgb(*cloudBufferRgb, *cloudBufferRgb, true);//ok
	pclUti->extractPlaneFromCloudRgb(*cloudBufferRgb, *cloudBufferRgb, true);//ok
	pclUti->extractPlaneFromCloudRgb(*cloudBufferRgb, *cloudBufferRgb, true);//ok
	cout << "preprocessing done ..." << endl;

	pclUti->doEuclideanClusteringRgb(*cloudBufferRgb, *cloudClusterRgb);
	*/
	//pclUti->doRegionGrowingSegmentationColor(*cloudBufferRgb, *cloudClusterRgb);//ok
	//pclUti->doRegionGrowingSegmentation(*rootPcl, *cloudClusterRgb); //ok

	//pclUti->testRandomSampleConsensus();// nok ???
	//pclUti->doConditionalEuclideanClusteringRgb(*cloudBufferRgb,*cloudBufferRgb);//NOK


	//pclUti->doEuclideanClusteringRgb(*cloudBufferRgb, *cloudClusterRgb);
	//pclUti->testDoEuclideanClusteringRgb(*cloudBufferRgb, *cloudClusterRgb);
	//pclUti->doCylinderSegmentationRgb(*cloudBufferRgb, *cloudClusterRgb);

	//pclUti->narfKeypointExtractionRgb(*cloudBufferRgb, *cloudClusterRgb);

	//pclUti->rotateZ(*rootPclRgb, *cloudBufferRgb);

	//pclUti->keypointExtractionNARF(*cloudBufferRgb, *cloudClusterRgb); //ok
	//pclUti->keypointExtractionISS(*cloudBufferRgb, *cloudClusterRgb); //ok
	//pclUti->keypointExtractionSIFT(*cloudBufferRgb, *cloudClusterRgb); //ok
	//pclUti->keypointExtractionHarris(*cloudBufferRgb, *cloudClusterRgb); //ok

	//pcl::copyPointCloud(*rootPclRgb,*cloudClusterRgb);
	/*
	pclUti->convertZPcl(*cloudClusterRgb);
	// Rotate
	theta = (float)((float)-180.0f*(float)((float)M_PI)/(float)180.0f);
	pclUti->rotateY(*cloudClusterRgb, *cloudClusterRgb, theta);
		theta = (float)((float)-180.0f*(float)((float)M_PI)/(float)180.0f);
	pclUti->rotateZ(*cloudClusterRgb, *cloudClusterRgb, theta);
	*/
	//pclUti->translationXYZ(*rootPclRgb, *cloudClusterRgb, 0, 0, 0);
	//pclUti->setPclRGBColor(*cloudClusterRgb, 255.0, 0, 0);
	//pclUti->FastPointFeatureHistogramsExample(*cloudBufferRgb, *cloudClusterRgb, *cloudDummyOutRgb); //ok - works not optimal

/*
	pclUti->voxelGrid(*rootPcl, *pclfilter1, 0.020f); //ok
	pclUti->voxelGrid(*rootPcl2, *pclfilter2, 0.020f); //ok

	pcl::PointCloud<pcl::PointNormal>::Ptr src_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr tar_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
	pclUti->getNormalsFromCloud(*pclfilter1, *src_normals_ptr);
	pclUti->getNormalsFromCloud(*pclfilter2, *tar_normals_ptr);

	pcl::PointCloud<pcl::PointWithScale>::Ptr src_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
	pclUti->keypointExtractionSIFT(*pclfilter1, src_normals_ptr, *src_keypoints_ptr);	//voxel grid error
	pcl::PointCloud<pcl::PointWithScale>::Ptr tar_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
	pclUti->keypointExtractionSIFT(*pclfilter2, tar_normals_ptr, *tar_keypoints_ptr);	
								 

 	Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
  	pclUti->extractFPFHfromKeypoints(*pclfilter1, *pclfilter2,
                                     *src_keypoints_ptr, *tar_keypoints_ptr,
									 *src_normals_ptr, *tar_normals_ptr,
									 tform);
	//pcl::transformPointCloud(*pclfilter1, *cloudBuffer, tform); //cloudBuffer = is tformed pclfilter1
	//pclUti->mergeXYZPcl(*cloudBuffer, *pclfilter2, cloudMerged);									 
*/
	Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
	pclUti->reconTwoPcl(*rootPcl, *rootPcl2, tform);
	//pcl::transformPointCloud(*rootPcl, *cloudBuffer, tform); //cloudBuffer = is tformed rootPCL
	//pclUti->mergeXYZPcl(*cloudBuffer, *rootPcl2, cloudMerged);

	pcl::transformPointCloud(*rootPclRgbFull, *cloudBufferRgb, tform); //cloudBuffer = is tformed rootPCL
	pclUti->mergeXYZRGBPcl(*cloudBufferRgb, *rootPclRgb2Full, cloudMergedRgb);


	////////////////////////////////////////////////////////////////////////////////////
	//update data
	rootPclResult = cloudMerged;
	rootPclRgbResult = cloudMergedRgb;
	pclApp::rootPclResultSize = (int)rootPclResult->size();
	pclApp::rootPclRgbResultSize = (int)rootPclRgbResult->size();
	////////////////////////////////////////////////////////////////////////////////////
	//Viewer selection
	pclUti->viewer = pclUti->viewportsDiffVisRgb(rootPclRgb, rootPclRgbResult);
	//pclUti->momentOfInertiaRgb(*cloudClusterRgb); //ok
	//pclUti->viewer = pclUti->viewportsDiffVis(rootPcl, rootPclResult);
	//pclUti->viewer = pclUti->simpleRangeImgVis(range_image_New);
	////////////////////////////////////////////////////////////////////////////////////
	//--------------------
	// -----Main loop-----
	//--------------------
	renderLoop();
}

/*
void pclApp::testPclObjectDetectionPipe01(){
	//////////////////////////////////////////
	//PCL normal
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudV (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtFilter (new pcl::PointCloud<pcl::PointXYZ>);

	//pclUti->voxelGrid(*rootPcl, *rootPcl, 0.04f);
	cout << "1 - rootPcl pcl Size: " << rootPcl->size() << endl;
	//pclUti->statisticalOutlierRemovalFilter(*rootPcl, *rootPcl, 280, 1.4);

	pclUti->radiusOutlierRemovalhelpFilter(*rootPcl, *cloudPtFilter, (int)0, (float)0.01, (int)4);

	cout << "2 - cloudPtFilter pcl Size: " << cloudPtFilter->size() << endl;
	////////////////////////////////////////////////////////////////////////////////////
	pclApp::rootPclResultSize = (int)rootPclRgb->size();
	pclUti->viewer = pclUti->simpleVis(cloudPtFilter);
	////////////////////////////////////////////////////////////////////////////////////
	//--------------------
	// -----Main loop-----
	//--------------------
	renderLoop();
}
*/


 /**
  * test line creater
  * input:\n
  * \param	float x, float y, float z
  * \return	void
  *
  */
void pclApp::testLineCreater(float x, float y, float z){	
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZRGB> axisPcl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr axisPcl (new pcl::PointCloud<pcl::PointXYZRGB>);
	//axisPcl->width = 20;
	//axisPcl->height = 20;
	axisPcl->is_dense = true;
	//axisPcl->points.resize(axisPcl->width * axisPcl->height);
	float ix = x; 
	float iy = y; 
	float iz = z; 
	float step = 0.01;

	int i=0;
	int imax=100;
	while(i<imax){
		pcl::PointXYZRGB axisPointX = pcl::PointXYZRGB(ix,y,z ,255,0,0);
		pcl::PointXYZRGB axisPointY = pcl::PointXYZRGB(x,iy,z ,0,255,0);
		pcl::PointXYZRGB axisPointZ = pcl::PointXYZRGB(x,y,iz ,0,0,255);
		axisPcl->push_back(axisPointX);
		ix = ix + step;
		axisPcl->push_back(axisPointY);
		iy = iy + step;
		axisPcl->push_back(axisPointZ);
		iz = iz + step;		
	i++;
	}
	//std::cout << "Resulting sift points are of size: " << axisPcl->size () <<std::endl;
	//pcl::io::savePLYFile("/media/hd2/dev/pcl/test.ply", *axisPcl);

	////////////////////////////////////////////////////////////////////////////////////
	//Viewer selection
	//pclUti->viewer = pclUti->viewRgb(axisPcl);
	pclUti->viewRgb(axisPcl);
	//pclUti->viewer = pclUti->rgbVis(rootPclRgb); 
	////////////////////////////////////////////////////////////////////////////////////
	renderLoop();
}

 /**
  * test Transform to Orientation to plane
  * \return	void
  *
  */
void pclApp::testTranformOrientationToPlane(){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclRgb1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclRgb2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dummy1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dummy2 (new pcl::PointCloud<pcl::PointXYZRGB>);

	pclUti->setWallrelativeToOriginXY(*rootPclRgbFull, *dummy1);
	//pclUti->setWallrelativeToOriginXY(*rootPclRgb2Full, *dummy2);
		//pclUti->createWallPlane(dummy1);
		//pclUti->mergeXYZRGBPcl(*rootPclRgbFull, *dummy1, pclRgb1);

	//pclUti->viewer = pclUti->rgbVis(dummy1);
	pclUti->viewer = pclUti->viewportsDiffVisRgb(rootPclRgbFull, dummy1);
	//pclUti->viewer = pclUti->viewportsDiffVisRgb(rootPclRgb2Full, dummy2);
	//pclUti->viewer = pclUti->rgbVisDouble(pclRgb1, pclRgb2);
	renderLoop();
}

 /**
  * test bigest plane orientation to plane
  * \return	void
  *
  */
void pclApp::testBigestPlaneOrientationToPlane(){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclRgb1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclRgb2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dummy1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dummy2 (new pcl::PointCloud<pcl::PointXYZRGB>);
}


 /**
  * start main with utility functions
  * \return	void
  *
  */
void pclApp::mainStart() {
//	pclUtilitys myPclUtilitysObj = pclUtilitys();
//	pclApp::pclUti = &myPclUtilitysObj;


	//convMatPclToPCL();

	//loadPlyTestPcl();

		//testViewsWithViewports();
		//testViewsWithNormals();
		//testPclObjectDetectionPipe01();
	    //testPclRgbObjectDetectionPipe01();
		//testLineCreater(2.0, 0, 0);
		//testTranformOrientationToPlane();

	//testObj.Run(strFilePath, Feature3d::Method::Harris); //ok sehr viele keypoints
	//testObj.Run(strFilePath, Feature3d::Method::ISS); //ok sehr viele keypoints auf den Sitzen weniger an der Wand,
	//testObj.Run(strFilePath, Feature3d::Method::SIFT); // ok sehr viele keypoints
}
#endif