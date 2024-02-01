/**
 * @file  main.cpp
 * @copyright Licensed under MIT
 * @brief main to show how to use the kv3 class.
 * @brief Author: Jörg Angermayer
 * @brief created on:  09.02.2023
 * @brief last update: 08.12.2023
 */
//============================================================================
// Name        : main.cpp
// Author      : Jörg Angermayer
// Version     : 1.0.0
// Date        : 20.02.2023
// Copyright   : MIT
// Description : kinect azure class
// MScode
// compile: F7
// run: <shift> + F5
//============================================================================

#include <iostream>
#include <unistd.h>
using namespace std;

#include "kv3.h"


static long int masterCounter = 0;
static int newImIsAvailable = 0;
static int viewMesh = 0;
static int smoothNormals = 0;
static bool bPointcloudIsEnabled = false;
string resultPath = string("/var/www/ramdev/");

#ifdef USE_ODAS
	
	#include <thread>
	//#include <pthread.h>
	//#include "kv3audiovisu.h"
	#include "kv3odas.h"
	#include "audioMeasurement.h"
	static int counterAudio = 0;
	static pthread_t thrKv3Cam;
	static pthread_t thrOdas;
#endif 

#ifdef USE_PCL
	#include "pclUtilitys.h"
	#include "pclApp.h"

	static bool bInitPclViewerSetupDone = false;
#endif 	

#ifdef USE_CV
/**
 * @brief addTextToImg
 * @brief add sensor values (serial nummber, temperature from cam and IMU, gravity, accelaration) into the image
 * @brief input: 
 * const cv::Mat (original image on which the text is to be written)
 * kv3 &kv3AppObj (kv3 handle)
 * @brief output: cv::Mat &out (original image with added text)
 */
void addkv3TextToImg(const cv::Mat &in, cv::Mat &out, kv3 &kv3AppObj ){
		out = in.clone();
		float capTemperature;
		float imuTemperature;
		kv3AppObj.getCapTemperature(capTemperature);
		kv3AppObj.getImuTemperature(imuTemperature);
		float gravX;
		float gravY;
		float gravZ;
		kv3AppObj.getGravity(gravX, gravY, gravZ);
		float accX;
		float accY;
		float accZ;
		kv3AppObj.getIMU(accX, accY, accZ);
		float accDegX;
		float accDegY;
		float accDegZ;

		//kv3AppObj.getIMUinDeg(accDegX, accDegY, accDegZ);
		kv3AppObj.getIMUinEuler(accDegX, accDegY, accDegZ);

		int xOffset = (int)(26);
		int yOffset = (int)(26);
		int xDelta = (int)(26);
		int yDelta = (int)(22);
		const float fontScale = 0.5;
		//const int txtOffset = 18;

		string strSerNr = "Serial Number: " + kv3AppObj.getSerialNumber();
		cv::putText(out, 
            strSerNr, //text
            cv::Point(xOffset, yOffset),
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
			1);		
		
		std::stringstream strStreamCapTemp;
		strStreamCapTemp << std::fixed << std::setprecision(1) << capTemperature;
		string strTempKv3 = "temp.kv3: " + strStreamCapTemp.str();//std::to_string(strStreamCapTemp);
		cv::putText(out, 
            strTempKv3, //text
            cv::Point(xOffset, yOffset + yDelta),
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
			1);

		std::stringstream strStreamImuTemp;
		strStreamImuTemp << std::fixed << std::setprecision(1) << imuTemperature;
		string strTempImu = "temp.IMU: " + strStreamImuTemp.str();
		cv::putText(out, 
            strTempImu, //text
            cv::Point(xOffset, yOffset + yDelta + yDelta),
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
			1);

		//Gravity
		int gravOffset = yOffset + yDelta + yDelta + yDelta;
		std::stringstream strStreamGravX;
		strStreamGravX << std::fixed << std::setprecision(3) << gravX;
		string strGravX= "Gravity X: " + strStreamGravX.str();
		cv::putText(out, 
            strGravX, //text
            cv::Point(xOffset, gravOffset),
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
			1);

		std::stringstream strStreamGravY;
		strStreamGravY << std::fixed << std::setprecision(3) << gravY;
		string strGravY= "Gravity Y: " + strStreamGravY.str();
		cv::putText(out, 
            strGravY, //text
            cv::Point(xOffset, gravOffset + yDelta),
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
			1);

		std::stringstream strStreamGravZ;
		strStreamGravZ << std::fixed << std::setprecision(3) << gravZ;
		string strGravZ= "Gravity Z: " + strStreamGravZ.str();
		cv::putText(out, 
            strGravZ, //text
            cv::Point(xOffset, gravOffset + yDelta + yDelta),
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
			1);			

		//IMU - ACC
		int accOffset = gravOffset + yDelta + yDelta + yDelta;
		std::stringstream strStreamAccX;
		std::stringstream strStreamAccDegX;  
		strStreamAccX << std::fixed << std::setprecision(3) << accX;
		strStreamAccDegX << std::fixed << std::setprecision(3) << accDegX;
		string strAccX = "IMU acc X: " + strStreamAccX.str() + " deg: " + strStreamAccDegX.str(); 
		//string strAccX = "Pitch in deg: " + strStreamAccDegX.str(); 
		cv::putText(out, 
            strAccX, //text
            cv::Point(xOffset, accOffset),
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
			1);

		std::stringstream strStreamAccY;
		std::stringstream strStreamAccDegY;
		strStreamAccY << std::fixed << std::setprecision(3) << accY;
		strStreamAccDegY << std::fixed << std::setprecision(3) << accDegY;
		string strAccY = "IMU acc Y: " + strStreamAccY.str() + " deg: " + strStreamAccDegY.str();
		//string strAccY = "Roll in deg: " + strStreamAccDegY.str();
		cv::putText(out, 
            strAccY, //text
            cv::Point(xOffset, accOffset + yDelta),
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
			1);		

		std::stringstream strStreamAccZ;
		std::stringstream strStreamAccDegZ;
		strStreamAccZ << std::fixed << std::setprecision(3) << accZ;
		strStreamAccDegZ << std::fixed << std::setprecision(3) << accDegZ;
		string strAccZ = "IMU acc Z: " + strStreamAccZ.str() + " deg: " + strStreamAccDegZ.str();
		//string strAccZ = "No Yaw in deg: " + strStreamAccDegZ.str();
		cv::putText(out, 
            strAccZ, //text
            cv::Point(xOffset, accOffset + yDelta + yDelta),
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
			1);	
}
#endif

#ifdef USE_CV
/**
 * @brief addkv3SetupTextToImg
 * @brief draw current kinect azure sensor setup values into the image
 * @brief input: 
 * const cv::Mat (original image on which the text is to be written)
 * kv3 &kv3AppObj (kv3 handle)
 * @brief output: cv::Mat &out (original image with added text)
 */
void addkv3SetupTextToImg(const cv::Mat &in, cv::Mat &out, kv3 &kv3AppObj){
	out = in.clone();

	int xOffset = (int)(26);
	int yOffset = (int)(16);
	int xDelta = 200;
	int yDelta = 22;
	
	const float fontScale = 0.5;
	//const int txtOffset = 18;
	string strFps;
	if(kv3AppObj.getFpsMode() == 0){ strFps = "5";}
	if(kv3AppObj.getFpsMode() == 1){ strFps = "15";}
	if(kv3AppObj.getFpsMode() == 2){ strFps = "30";}

	int id = kv3AppObj.getCurrentSetupId();
	string strSetupId = string("Setup ID: " + to_string(id) + " - " + strFps + " FPS");
	int rgbX = 0; int rgbY = 0;
	kv3AppObj.getRgbResolution(rgbX, rgbY);
	string strSetupRGBInfo = "RGB: " + to_string(rgbX) + "x" + to_string(rgbY);
	int depthX = 0; int depthY = 0;
	kv3AppObj.getDepthFieldResolution(depthX, depthY);
	string strSetupDepthInfo = "Depth: " + to_string(depthX) + "x" + to_string(depthY);
		
	cv::putText(out, 
		strSetupId, //text
		cv::Point(xOffset, yOffset),
		cv::FONT_HERSHEY_SIMPLEX,
		fontScale,
		CV_RGB(118, 185, 0),
		1);		

	cv::putText(out, 
		strSetupRGBInfo, //text
		cv::Point(xOffset+xDelta, yOffset),
		cv::FONT_HERSHEY_SIMPLEX,
		fontScale,
		CV_RGB(118, 185, 0),
		1);	

	cv::putText(out, 
		strSetupDepthInfo, //text
		cv::Point(xOffset+xDelta, yOffset+yDelta),
		cv::FONT_HERSHEY_SIMPLEX,
		fontScale,
		CV_RGB(118, 185, 0),
		1);	
}
#endif

#ifdef USE_CV
#ifdef USE_PCL
void convertPclRgbToVec(pcl::PointCloud<pcl::PointXYZRGB>& in, vector<Point3f> &vecPoint3D, vector<Point3i> &vecRgb, float scale, bool bCoordSwitch){	

	int i=0;
	int imax = in.points.size();

	if(bCoordSwitch == true){
		while(i<imax){
			Point3f Po;
			Po.x = in.points[i].x*scale;
			Po.y = in.points[i].z*scale;
			Po.z = (in.points[i].y*scale) * (-1);
			vecPoint3D.push_back(Po);
			Point3i Prgb;
			Prgb.x = in.points[i].r;
			Prgb.y = in.points[i].g;
			Prgb.z = in.points[i].b;
			vecRgb.push_back(Prgb);
			i++;
		}	
	}else{
		while(i<imax){
			Point3f Po;
			Po.y = in.points[i].x*scale;
			Po.x = in.points[i].z*scale;
			Po.z = (in.points[i].y*scale) * (-1);	
			vecPoint3D.push_back(Po);
			Point3i Prgb;
			Prgb.x = in.points[i].r;
			Prgb.y = in.points[i].g;
			Prgb.z = in.points[i].b;
			vecRgb.push_back(Prgb);
			i++;
		}	

	}
}
#endif
#endif

#ifdef USE_CV
#ifdef USE_ODAS
 /**
  * show image windows with kinect v3 data, audio data and ODAS data 
  * input:\n
  * \param	kv3odas object audioMeasurement object 
  * \return	void
  *
  */
void showKv2Demo(kv3odas *kv3AppOdas , audioMeasurement *audioM ){	
	//kv3audiovisu kv3AppAudiovisuObj = kv3audiovisu(); 

	//AudioMeasurement
	string strWin05 = string("Audio Measurement");

	//ODAS	
	int idx;
	float x, y, z, len;
	string strWin04 = string("ODAS - Open embeddeD Audio System");		

	//kv3
	string strWin01 = string("Resized and registered color and depth image - CV_8UC4 - 24");
	string strWin02 = string("Original Ir(left) and depth(right) image - CV_16UC1 - 2");
	
	float factor = 1.4;
	int w = round(640/factor);
	int h = round(576/factor);
	int h2 = 0;

		Mat colorImg;
		Mat irImg;
		Mat depthImg;
		Mat colorImgRegDepth;
		Mat kv3PCL;
		Mat audioMimg;

	int winx1 = 0;
	int winy1 = 0;
	
	kv3 kv3AppObj = kv3();
	kv3AppObj.setup03();

	cout << "start update loop" << endl;
	Mat win02(h, w*2, CV_16UC1, Scalar(0,0,0));

	while(1){	
		if(audioM != nullptr){
			audioM->update();
			audioMimg = audioM->getMeasurementImage();
		}
		
		kv3AppObj.update(0);

		kv3AppObj.getCvColorImg(colorImg);
		kv3AppObj.getCvIrImg(irImg);
		kv3AppObj.getCvDepthImg(depthImg);
		kv3AppObj.getCvRegDepthImg(colorImgRegDepth);		

		kv3AppObj.resizeImg(colorImg, w, 0);
		kv3AppObj.resizeImg(colorImgRegDepth, w, 0);
		h2 = colorImg.size().height;
		Mat win01(h2, w*2, CV_8UC4, Scalar(0,0,0));
		kv3AppObj.resizeImg(irImg, w, h);
		kv3AppObj.resizeImg(depthImg, w, h);
		
		kv3AppObj.convToVisibleImg(colorImgRegDepth, colorImgRegDepth);
		kv3AppObj.convIrToVisibleImg(irImg, irImg);
		kv3AppObj.convToVisibleImg(depthImg, depthImg);

	cv::namedWindow( strWin01, cv::WINDOW_AUTOSIZE|cv::WINDOW_GUI_NORMAL);
	cv::namedWindow( strWin02, cv::WINDOW_AUTOSIZE|cv::WINDOW_GUI_NORMAL);
	if(kv3AppOdas != nullptr){
		cv::namedWindow( strWin04, cv::WINDOW_AUTOSIZE|cv::WINDOW_GUI_NORMAL);
	}
	if(audioM != nullptr){
		cv::namedWindow( strWin05, cv::WINDOW_AUTOSIZE|cv::WINDOW_GUI_NORMAL);		
	}		
	
		//win 01
		cv::Mat regDepth8UC4;
		colorImgRegDepth.convertTo(regDepth8UC4, CV_8UC4, 1.0 / 255.0, 0.0);
		cvtColor(regDepth8UC4, regDepth8UC4, cv::COLOR_RGB2RGBA);
		addkv3TextToImg(regDepth8UC4, regDepth8UC4, kv3AppObj);
		
		//regDepth8UC4.copyTo(win01(Rect(w, 0, regDepth8UC4.size().width, h2)));
		
		Mat colorImg2;
		cvtColor(colorImg, colorImg2, cv::COLOR_RGB2RGBA);
		colorImg.copyTo(win01(Rect(0, 0, colorImg.size().width, h2)));	

		//win 02
		imshow(strWin01,win01);
		moveWindow(strWin01, 0, 0);

		irImg.copyTo(win02(Rect(0, 0, irImg.size().width, irImg.size().height)));
		depthImg.copyTo(win02(Rect(w, 0, depthImg.size().width, depthImg.size().height)));
		imshow(strWin02,win02);
		moveWindow(strWin02, 0, win01.size().height+64);

		//ODAS
		if(kv3AppOdas != nullptr){		
			kv3AppOdas->getNearesSoundObj(idx, x, y, z, len);
			Mat trImg = kv3AppOdas->drawOdasTrackingData(idx, x, y, z);
			imshow(strWin04,trImg);
			moveWindow(strWin04, w+w+72, win01.size().height+64);	
		}

		//Audio Measurement
		if(audioM != nullptr){
			imshow(strWin05, audioMimg);
			moveWindow(strWin05, 0, h2+win02.size().height+64+64);
		}

		char key = waitKey(10);
        if(key == 27){			
			cv::destroyWindow( strWin01 ); 
			cv::destroyWindow( strWin02 ); 
			if(kv3AppOdas != nullptr){
				cv::destroyWindow( strWin04 ); 	
			}
			if(audioM != nullptr){
				cv::destroyWindow( strWin05 ); 
			}
			break; }
	}	

}
#endif
#endif


#ifdef USE_CV
 /**
  * test crop image
  * input:\n
  * \param	kv3 object
  * \return	void
  *
  */
void test(kv3 &kv3AppObj){
	Mat cropImgColor;
	Mat cropImgIr;
	Mat cropImgDepth;
	Mat resizeImgColor;
	Mat resizeImgDepth;
	Mat conCropImgIr;
	Mat conCropImg8Ir;
	kv3AppObj.getCvIrImg(cropImgIr);
	if(cropImgIr.empty() == false){
		kv3AppObj.convIrToVisibleImg(cropImgIr, conCropImgIr);
		if(conCropImgIr.empty() == false){
			imshow("conCropImgIr", conCropImgIr);
		}else{
			cout << "conCropImgIr is empty..." << endl;
		}

		kv3AppObj.convIrToVisible8BitGrayImg(cropImgIr, conCropImg8Ir);
		if(conCropImg8Ir.empty() == false){
			imshow("conCropImg8Ir", conCropImg8Ir);
		}else{
			cout << "conCropImg8Ir is empty..." << endl;
		}
		

	}else{
		cout << "cropImgIr is empty..." << endl;
	}
}
#endif

#ifdef USE_CV
 /**
  * grab kv3 camera
  * \return	void
  *
  */
void *kv3Cam(void *ptr){
	kv3 kv3AppObj = kv3();
	kv3AppObj.setup01();
    //kv3AppObj.testloop();
	while(1){
		kv3AppObj.update(0);
		test(kv3AppObj);
		char key = waitKey(10);
        if(key == 27){ break; }
	}	
}
#endif

#ifdef USE_CV
 /**
  * update kv3 data and use a function to show data inside windows
  * \return	void
  *
  */
void kv3RunCam(){
	kv3 kv3AppObj = kv3();
	kv3AppObj.setup01();
	while(1){
		kv3AppObj.update(0);
		test(kv3AppObj);

/*
		float capTemperature;
		float imuTemperature;
		kv3AppObj.getCapTemperature(capTemperature);
		kv3AppObj.getImuTemperature(imuTemperature);
		cout << "capTemperature: " << capTemperature << " imuTemperature: " << imuTemperature << endl;
*/

		char key = waitKey(10);
        if(key == 27){ break; }
	}	
}
#endif


#ifdef USE_ODAS
 /**
  * run ODAS server
  * \return	void
  *
  */
void kv3RunOdasServer(){
	kv3odas kv3AppOdas = kv3odas(9000);
	kv3AppOdas.odasServer();
}

 /**
  * create ODAS server function pointer
  * \return	void
  *
  */
void *kv3OdasServer(void *ptr){
	kv3odas *kv3AppOdasPtr;
	kv3AppOdasPtr = (kv3odas *)ptr;
	kv3AppOdasPtr->odasServer();
}

#ifdef USE_CV
 /**
  * test sync between kv3 image data and  ODAS data
  * input:\n
  * \param	kv3odas object
  * \return	void
  *
  */
void testSyncStart(kv3odas kv3AppOdas){
	kv3audiovisu kv3AppAudiovisuObj = kv3audiovisu();
	int idx;
	float x;
	float y;
	float z;
	float len;

	kv3 kv3AppObj = kv3();
	kv3AppObj.setup03();
	while(1){
		kv3AppObj.update(0);

			kv3AppOdas.getNearesSoundObj(idx, x, y, z, len);
			//cout << "idx: " << idx << " x: " << x << " y: " << y << " z: " << z << " len: " << len << endl;
			Mat trImg = kv3AppAudiovisuObj.drawOdasTrackingData(idx, x, y, z);
			imshow("trImg",trImg);

		test(kv3AppObj);
		//float capTemperature;
		//kv3AppObj.getCapTemperature(capTemperature);
		//cout << "capTemperature: " << capTemperature << endl;
		char key = waitKey(10);
        if(key == 27){ break; }
	}	
}
#endif
#endif


 /**
  * main function
  * \return	void
  *
  */
int main() {
	kv3 kv3AppObj = kv3();
	kv3AppObj.setup03();
	int kv3PipelineMode = 0;

	// windows parameter
	int imgWinWidth = 400;//400
	int widthOfPcl = 512;//512
	int showImgWidth = 512;//512
	int borDis = 50; // border distance

	int winCC_x = imgWinWidth+imgWinWidth+borDis+borDis;
	int winCC_y = 920;//880;

#ifdef USE_ODAS
	// this app is starting the odas server
	// start the odas live system with the configuration for kinect azure to get online the needed data:
	//  cd /media/hd2/git_pool_hd2/ODASpool/odas/build/bin
	//  sudo ./odaslive -vc /media/hd2/git_pool_hd2/ODASpool/odas/config/odaslive/kv3Socket.cfg
	kv3odas kv3AppOdas = kv3odas(9000);
	pthread_create(&thrOdas, NULL, kv3OdasServer, (kv3odas *)&kv3AppOdas);
	kv3AppOdas.startOdasLive();// try to start the odas live system, but it need sudo rights to work.
	string strWinOdas = string("ODAS - Open embeddeD Audio System");	
#endif	

#ifdef USE_PCL
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbPcl (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::visualization::PCLVisualizer viewer ("Viewer");
	pcl::PolygonMesh liveMesh;

	viewer.setBackgroundColor(0,0,0);
	viewer.setSize(800,winCC_y);
	viewer.setPosition(winCC_x+100,0);
	viewer.addPointCloud<pcl::PointXYZRGB> (rgbPcl,"rgbPcl");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rgbPcl"); 
	pclUtilitys myPclUtily = pclUtilitys();
#endif
	
#ifdef USE_CV
		Mat colorImg;
		Mat colorImgRegDepth;
		Mat depthImg;
		Mat irImg;		

		Mat resizeImgColor;
		Mat resizeImgColorDepth;
		Mat resizeImgDepth;
		Mat resizeImgIr;

		kv3AppObj.update(kv3PipelineMode);	
		
		if(kv3PipelineMode == 0){				
			kv3AppObj.getCvColorImg(colorImg);		
			kv3AppObj.getCvDepthImg(depthImg);
			kv3AppObj.getCvRegDepthImg(colorImgRegDepth);
			kv3AppObj.getCvIrImg(irImg);
			resizeImgColor = colorImg.clone();
			resizeImgColorDepth = depthImg.clone();
			resizeImgDepth = depthImg.clone();
			resizeImgIr = irImg.clone();
		}
		if(kv3PipelineMode == 1){											
			kv3AppObj.getCvRegDepthImg(colorImgRegDepth);
			kv3AppObj.getCvDepthImg(depthImg);
			resizeImgColorDepth = colorImgRegDepth.clone();
			resizeImgDepth = depthImg.clone();
		}		
#endif 

#ifdef USE_CVGUI

Mat winCC(winCC_y, winCC_x, CV_8UC3, Scalar(0,0,0));
cv::namedWindow( "Control Center", cv::WINDOW_AUTOSIZE|cv::WINDOW_GUI_NORMAL);
//Mat imgBg = winCC.clone();

char cwd[256];
getcwd(cwd, sizeof(cwd));
std::cout << " ---- Current path is " << cwd << '\n';
	
	//string pathToBg = String("/media/hd2/dev/60_kinectV3/01_work/kv3EstPose/assets/bg_1920_rusty.jpg");
	string pathToBg = String("../../assets/bg_1920_rusty.jpg");
	Mat imgBg = imread(pathToBg);
	if(!imgBg.empty()){
		kv3AppObj.resizeImg(imgBg,winCC_x,winCC_y);	
		imgBg.copyTo(winCC(Rect(0, 0, imgBg.size().width, imgBg.size().height)));
	}else{
		imgBg = winCC.clone();
	}
	
	int imgColorPosX = borDis;
	int imgColorPosY = borDis+borDis;
	//int imgColorWidth = 400;
	int imgColorHeight = 300;
	int imgIrPosX = 0;
	int imgIrPosY = borDis;
	int imgIrWidth = 0;
	int imgIrHeight = 0;
	int imgDepthPosX = 0;
	int imgDepthPosY = borDis;	
	int imgDepthWidth = 0;
	int imgDepthHeight = 0;	
#endif


	int currentDepthFieldX; int currentDepthFieldY;
	while(1){
		newImIsAvailable = 0;				

		int updateResult = kv3AppObj.update(kv3PipelineMode);
		if(updateResult == -1){
			cout << "--- Error --- updateResult: " << updateResult << endl;
		}else{
			#ifdef USE_CV
			if(kv3PipelineMode == 0){
					
				kv3AppObj.getCvColorImg(colorImg);											
				kv3AppObj.getCvRegDepthImg(colorImgRegDepth);
				kv3AppObj.getCvDepthImg(depthImg);
				kv3AppObj.getCvIrImg(irImg);
				resizeImgColor = colorImg.clone();
				resizeImgColorDepth = colorImgRegDepth.clone();
				resizeImgDepth = depthImg.clone();
				resizeImgIr = irImg.clone();	

				if(!resizeImgColor.empty()){
					int curImgBlurLevel =  kv3AppObj.detectCurrentImageBlurStatusViaFFT(resizeImgColor, 2);//	
					if(curImgBlurLevel == 0){
						newImIsAvailable = 1;
					}else{
						//cout << "image are to blurry ..." << endl;
					}		
				}	
				
			}		
			if(kv3PipelineMode == 1){			
				kv3AppObj.getCvRegDepthImg(colorImgRegDepth);
				kv3AppObj.getCvDepthImg(depthImg);
				resizeImgColorDepth = colorImgRegDepth.clone();
				resizeImgDepth = depthImg.clone();	

				if(!resizeImgColorDepth.empty()){
					int curImgBlurLevel =  kv3AppObj.detectCurrentImageBlurStatusViaFFT(resizeImgColorDepth, -8);	
					if(curImgBlurLevel == 0){
						newImIsAvailable = 1;
					}else{
						//cout << "image are to blurry ..." << endl;
					}		
				}					
			}	

		#ifdef USE_ODAS
			cv::namedWindow( strWinOdas, cv::WINDOW_AUTOSIZE|cv::WINDOW_GUI_NORMAL);
			int idx;
			float x; float y; float z; float len;
			kv3AppOdas.getNearesSoundObj(idx, x, y, z, len);
			Mat trImg = kv3AppOdas.drawOdasTrackingData(idx, x, y, z);
			imshow(strWinOdas,trImg);	
			moveWindow(strWinOdas, winCC_x+100,0);
		#endif		

			#endif	

		}//end_if_else
	
		///Show
		#ifdef USE_CVGUI
		if(!winCC.empty()){

			//Information about active sensor mode
			int setupInfoPanelWidth = winCC.size().width-(borDis+borDis);
			Mat imgkv3SetupData(borDis, setupInfoPanelWidth, CV_8UC3, Scalar(0,0,0));
			addkv3SetupTextToImg(imgkv3SetupData, imgkv3SetupData, kv3AppObj);		
			imgkv3SetupData.copyTo(winCC(Rect(borDis, borDis, imgkv3SetupData.size().width, imgkv3SetupData.size().height)));		

			if(updateResult != 0){
				// Sensor and IMU data
				int deltaImgSize = imgWinWidth;
				Mat imgkv3data(imgColorHeight, deltaImgSize, CV_8UC3, Scalar(0,0,0));
				imgkv3data = cv::Mat::zeros( imgkv3data.size(), imgkv3data.type() );	
				addkv3TextToImg(imgkv3data, imgkv3data, kv3AppObj);
				imgkv3data.copyTo(winCC(Rect(borDis+imgWinWidth, imgColorPosY, imgkv3data.size().width, imgkv3data.size().height)));			
			}

			//if((updateResult != 0)&&(newImIsAvailable == 1)&&(kv3PipelineMode == 0)){	
			if((updateResult != 0)&&(newImIsAvailable == 1)){	
					if(kv3PipelineMode == 0){
						if(!resizeImgColor.empty()){	
							kv3AppObj.resizeImg(resizeImgColor, imgWinWidth,0);	
							cvtColor(resizeImgColor,resizeImgColor,cv::COLOR_BGRA2BGR);
							resizeImgColor.copyTo(winCC(Rect(imgColorPosX, imgColorPosY, resizeImgColor.size().width, resizeImgColor.size().height)));
							cv::Rect rectRGB(imgColorPosX, imgColorPosY, resizeImgColor.size().width, resizeImgColor.size().height);
							rectangle(winCC, rectRGB, cv::Scalar(100, 100, 100), 1);
							//imgColorWidth = resizeImgColor.size().width;
							imgColorHeight = resizeImgColor.size().height;
						}
					}
					if(kv3PipelineMode == 1){
						if(!resizeImgColorDepth.empty()){	
							kv3AppObj.resizeImg(resizeImgColorDepth, imgWinWidth,0);	
							cvtColor(resizeImgColorDepth,resizeImgColorDepth,cv::COLOR_BGRA2BGR);
							resizeImgColorDepth.copyTo(winCC(Rect(imgColorPosX, imgColorPosY, resizeImgColorDepth.size().width, resizeImgColorDepth.size().height)));
							cv::Rect rectRGB(imgColorPosX, imgColorPosY, resizeImgColorDepth.size().width, resizeImgColorDepth.size().height);
							rectangle(winCC, rectRGB, cv::Scalar(100, 100, 100), 1);
							//imgColorWidth = resizeImgColor.size().width;
							imgColorHeight = resizeImgColorDepth.size().height;
						}
					}					

					if(!resizeImgIr.empty()){
						kv3AppObj.resizeImg(resizeImgIr,imgWinWidth,0);	
						//kv3AppObj.convIrToVisibleImg(resizeImgIr, resizeImgIr);
						resizeImgIr.convertTo(resizeImgIr, CV_8U, 1 / 255.0);				
						cvtColor(resizeImgIr,resizeImgIr,cv::COLOR_GRAY2BGR);
						imgIrPosX = borDis;
						imgIrPosY = imgColorHeight + borDis + borDis;
						resizeImgIr.copyTo(winCC(Rect(imgIrPosX, imgIrPosY, resizeImgIr.size().width, resizeImgIr.size().height)));
						cv::Rect rectIr(imgIrPosX, imgIrPosY, resizeImgIr.size().width, resizeImgIr.size().height);
						rectangle(winCC, rectIr, cv::Scalar(100, 100, 100), 1);
						imgIrWidth = resizeImgIr.size().width;
						imgIrHeight = resizeImgIr.size().height;
					}
					if(!resizeImgDepth.empty()){	
						kv3AppObj.resizeImg(resizeImgDepth,imgWinWidth,0);
						kv3AppObj.convToVisibleImg(resizeImgDepth, resizeImgDepth);
						resizeImgDepth.convertTo(resizeImgDepth, CV_8U, 1 / 256.0, 0.0);
						cvtColor(resizeImgDepth,resizeImgDepth,cv::COLOR_GRAY2BGR);
						if(kv3PipelineMode == 0){
							imgDepthPosX = imgIrPosX + imgWinWidth;
							imgDepthPosY = imgIrPosY;
						}
						if(kv3PipelineMode == 1){
							imgDepthPosX = borDis;
							imgDepthPosY = imgColorHeight + borDis + borDis;
						}
						resizeImgDepth.copyTo(winCC(Rect(imgDepthPosX, imgDepthPosY, resizeImgDepth.size().width, resizeImgDepth.size().height)));
						cv::Rect rectDepth(imgDepthPosX, imgDepthPosY, resizeImgDepth.size().width, resizeImgDepth.size().height);
						rectangle(winCC, rectDepth, cv::Scalar(100, 100, 100), 1);
						imgDepthWidth = resizeImgDepth.size().width;
						imgDepthHeight = resizeImgDepth.size().height;				
					}		

			}//end_if(updateResult != 0){
			imshow("Control Center", winCC);
			moveWindow("Control Center", 0, 0);
		}

		//Viz3dObj.spinOnce(1, true);// richtige Platz

		char key = waitKey(1);
        if(key == 27){ 
			destroyAllWindows();
			break; }

        if(key == '1'){ 		
			kv3AppObj.setup03();
			imgBg.copyTo(winCC(Rect(0, 0, imgBg.size().width, imgBg.size().height)));
			int depthFieldX;int depthFieldY;
			kv3AppObj.getDepthFieldResolution(depthFieldX, depthFieldY);
			widthOfPcl = 512;//depthFieldX/2;									
		 }//end if key()		
        if(key == '2'){ 				
			kv3AppObj.setup02();
			imgBg.copyTo(winCC(Rect(0, 0, imgBg.size().width, imgBg.size().height)));
			int depthFieldX;int depthFieldY;
			kv3AppObj.getDepthFieldResolution(depthFieldX, depthFieldY);
			widthOfPcl = 512;//depthFieldX/2;
		 }//end if key()
        if(key == '3'){ 	
			kv3AppObj.setup01();			
			imgBg.copyTo(winCC(Rect(0, 0, imgBg.size().width, imgBg.size().height)));
			int depthFieldX;int depthFieldY;
			kv3AppObj.getDepthFieldResolution(depthFieldX, depthFieldY);
			widthOfPcl = 512;//depthFieldX/2;		
		 }//end if key()
        if(key == '4'){ 				
			kv3AppObj.setup04();
			imgBg.copyTo(winCC(Rect(0, 0, imgBg.size().width, imgBg.size().height)));
			int depthFieldX;int depthFieldY;
			kv3AppObj.getDepthFieldResolution(depthFieldX, depthFieldY);
			widthOfPcl = 512;//depthFieldX;			
		 }//end if key()		

        if(key == '5'){ 				
			kv3AppObj.setup05();
			imgBg.copyTo(winCC(Rect(0, 0, imgBg.size().width, imgBg.size().height)));
			int depthFieldX;int depthFieldY;
			kv3AppObj.getDepthFieldResolution(depthFieldX, depthFieldY);
			widthOfPcl = 512;//depthFieldX;			
		 }//end if key()		  		 		 		

        if((key == 's')&&(newImIsAvailable == 1)){ 		
			Mat grabPcl;
			kv3AppObj.calcCvPclCamColor(depthImg, grabPcl);
			cv::viz::writeCloud(string(resultPath+to_string(masterCounter)+"_kv3_CV2viz_Pcl.ply").c_str(), grabPcl, colorImgRegDepth);		
			#ifdef USE_ESTPOSE	
			objEstPose.savekv3CameraPose(resultPath, masterCounter); //ok
			#endif
			kv3AppObj.savekv3data(resultPath, masterCounter); //ok
			kv3AppObj.saveColorImg(resultPath, masterCounter, colorImg);//ok
			kv3AppObj.saveRGBDImg(resultPath, masterCounter, colorImgRegDepth,depthImg);//ok	
			kv3AppObj.writePCL(string(resultPath+to_string(masterCounter)+"_Pcl.ply").c_str()); //ok
			kv3AppObj.writePclRgb(string(resultPath+to_string(masterCounter)+"_PclRgb.ply").c_str());//ok	
			kv3AppObj.writePclRgbAsPCDfile(string(resultPath+to_string(masterCounter)+"_PclRgb.pcd").c_str());// 

			masterCounter++;
		}//end if key()	
		if(key == 'j'){ 
			kv3AppObj.setImageFormat(1);//mjpeg
			kv3AppObj.setCurrentSetup(kv3AppObj.getCurrentSetupId());		
		}	 
		if(key == 'b'){ 
			kv3AppObj.setImageFormat(0);//BGRA
			kv3AppObj.setCurrentSetup(kv3AppObj.getCurrentSetupId());	
		}		

		#ifdef USE_PCL
		//Pointcloud RGB
		if(key == 'l'){ 
			//meshing
			viewer.removePolygonMesh("liveMesh");	
			if(rgbPcl->size() > 100){
				cout << "start --- meshingBsplineFitting ... " << endl;
				myPclUtily.meshingBsplineFitting(rgbPcl, liveMesh);
				cout << "end --- meshingBsplineFitting ... " << endl;
				//pcl::io::savePolygonFileSTL("/var/www/ramdev/bSplineFitting.stl", liveMesh);
				pcl::io::saveOBJFile(string(resultPath+to_string(masterCounter)+"_kv3_Mesh_bSplineFitting.obj").c_str(), liveMesh);
			}		
		}

		if(key == 'n'){ 
			if(smoothNormals == 0){
				smoothNormals = 1;
			}else{
				smoothNormals = 0;
			}
		}	

		if(key == 'p'){ 
			if(bPointcloudIsEnabled == false){
				bPointcloudIsEnabled = true;				
			}else{
				bPointcloudIsEnabled = false;
				viewMesh = 0;
			}
		}	

		//if(kv3AppObj.getCurrentSetupId() == 4){	

		//}else{
			if(bPointcloudIsEnabled == true){
					rgbPcl->clear();
					kv3AppObj.updatePclRgbforPclLib(rgbPcl);
					myPclUtily.passTroughFilterRgb(*rgbPcl, *rgbPcl, 0.2, 3.00);
				//myPclUtily.passTroughFilterRgb(*rgbPcl, *rgbPcl, 0.55, 1.00);
				//myPclUtily.cropFilterRgb(*rgbPcl, *rgbPcl, -0.250, 0.250, -0.50, 0.50);
				myPclUtily.voxelGridRgb(*rgbPcl, *rgbPcl, 0.006);//0.010
					//cout << "rgbPcl size: " << rgbPcl->size() << endl;
				myPclUtily.statisticalOutlierRemovalFilterRgb(*rgbPcl, *rgbPcl, 60.0, 0.15);
					//cout << "___________rgbPcl size: " << rgbPcl->size() << endl;
					myPclUtily.rotateX(*rgbPcl, *rgbPcl, 135);
				
				//switch between pointcloud view and mesh view
				if(key == 'm'){ 	
					if(viewMesh == 1){
						viewMesh = 0;
						viewer.removePolygonMesh("liveMesh");
						viewer.addPointCloud<pcl::PointXYZRGB> (rgbPcl,"rgbPcl");
						viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rgbPcl"); 
						kv3AppObj.updatePclRgbforPclLib(rgbPcl);
					}else{
						viewMesh = 1;
						rgbPcl->clear();
						viewer.removeAllPointClouds();
					}
				}	

				if(viewMesh == 0){
					viewer.updatePointCloud(rgbPcl,"rgbPcl");
				}else{
					//meshing
					viewer.removePolygonMesh("liveMesh");	

					if(rgbPcl->size() > 100){
						myPclUtily.meshingViaFastTriangulation(rgbPcl, liveMesh, smoothNormals);
						//myPclUtily.meshingViaPoisson(rgbPcl, liveMesh, smoothNormals);
					}
					viewer.addPolygonMesh(liveMesh,"liveMesh");
				}
			//}//end_if bPointcloudIsEnabled
		}//if kv3AppObj.getCurrentSetupId() 
	
		viewer.spinOnce(0, true);

		#endif
		#endif

		kv3AppObj.releaseAllkv3img();
	}//end while
	cout << "---------- while loop exit" << endl;
	kv3AppObj.releaseCapture();

#ifdef USE_ODAS
	//  cd /media/hd2/git_pool_hd2/ODASpool/odas/build/bin
	//  sudo ./odaslive -vc /media/hd2/git_pool_hd2/ODASpool/odas/config/odaslive/kv3Socket.cfg

	//kv3RunCam();

/*
	kv3odas kv3AppOdas = kv3odas(9000);
	pthread_create(&thrOdas, NULL, kv3OdasServer, (kv3odas *)&kv3AppOdas);
	kv3AppOdas.startOdasLive();

	if(kv3AppOdas != nullptr){
		string strWinOdas = string("ODAS - Open embeddeD Audio System");
		cv::namedWindow( strWinOdas, cv::WINDOW_AUTOSIZE|cv::WINDOW_GUI_NORMAL);

		kv3AppOdas->getNearesSoundObj(idx, x, y, z, len);
		Mat trImg = kv3AppOdas->drawOdasTrackingData(idx, x, y, z);
		imshow(strWinOdas,trImg);
		//moveWindow(strWinOdas, w+w+72, win01.size().height+64);	
	}
*/
	//audioMeasurement audioM = audioMeasurement();
	//showKv2Demo(&kv3AppOdas, &audioM);
	//showKv2Demo(nullptr, &audioM);
	//showKv2Demo(&kv3AppOdas, nullptr);

#endif

	return 0;
}
