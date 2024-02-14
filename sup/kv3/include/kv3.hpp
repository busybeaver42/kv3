/**
* @file kv3.h
* @brief Implementation of the class functions to access the kinect azure sensor set data.
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

#ifndef KV3_HPP_
#define KV3_HPP_

#include <iostream>
#include <thread>
#include <fstream>
#include <sstream>
#include <cstring>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <cstdlib>
#include <signal.h>
#include <math.h>
#include <vector>
#include <array>
#include <map>
#include <iomanip> //for std::setprecision 
using namespace std;

#include <k4arecord/playback.h>
#include <k4a/k4a.h>
#include <k4a/k4atypes.h>
//#include <k4abt.h>
#include "../include/Vector.h"

#ifdef USE_CV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/dnn.hpp>
using namespace cv;
#endif

#ifdef USE_PCL
	#include <pclApp.h>
#endif

struct color_point_t
{
    int16_t xyz[3];
    uint8_t rgb[3];
};

/**
 *  kv3 class. kinect azure sensor set framework class.
 */
class kv3 {
	public:
	      /**
       * A constructor.
       * A more elaborate description of the constructor.
       */
		kv3();
		~kv3();
		void setupImFormatAndFPSconfig();
        void setup01(); //RGB best support (1280x720 MJPEG/YUY2/NV12) - 30 FPS - depth 640x576; 0.5 - 3.86 m
		void setup02(); //RGB highest resolution (3840x2160) - 15 FPS - depth 640x576; 0.5 - 3.86 m
		void setup03(); //RGB best support (1920x1080) - 15 FPS - depth 1024x1024 ; 0.25 - 2.21 m
		void setup04(); //RGB best support (1280x720 MJPEG/YUY2/NV12) - 30 FPS - long range depth 320x288 ; 0.5 - 5.46 m
		void setup05(); //RGB resolution (3840x2160) - 15 FPS - depth 640x576; 0.50 - 3.86 m
		void setup06(); //RGB 4:3 resolution (2048x1536) - 15 FPS - depth 512x512; 0.25 - 2.88 m
		void setup07(); //RGB 4:3 highest resolution (4096x3072) -  5 FPS - depth 512x512; 0.25 - 2.88 m 

		void getDepthFieldResolution(int &x, int &y);
		void getRgbResolution(int &x, int &y);
 		void setupCalib();
		int getCurrentSetupId();
		void setCurrentSetup(int id);
		void create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table);
		#ifdef USE_PCL
		void updatePclforPclLib(pcl::PointCloud<pcl::PointXYZ>::Ptr& outPcl, k4a_image_t k4aPcl);
		void updatePclRgbforPclLib(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outPcl);
		#endif
		
		void writePCL(const char *file_name);
		void writePclRgb(const char *file_name);
		void writePclRgbAsPCDfile(const char *file_name);
		//void write_color_point_cloud(string path);
		void writeColorPcl1(const char *file_name); 
		//void writeColorPcl2(const char *file_name);
		void readCurrentColorMode();
		void setImageFormat(int format);//0=BGRA;1=MJPEG
		void setFpsMode(int mode);
		int getFpsMode();
        int update(int mode);
		void releaseAllkv3img();
		void updateIMU();
		void capTemperature();	
		void shutdown();
		void releaseCapture();
        void testloop();
		#ifdef USE_CV
		void capColor();
		void capColorReg();
		void capColorRegMJPEG();
		void capColorMJPEG();
		void capDepth();
		void capIr();
		void capPclDepth();
		void getPcl3DPointFrom2DPoint(const Point &inPo2D, Point3d &outPclPoint);
		bool transformColor2DpointToDepth2Dpoint(const Point &inPo2D, Point &outPo2D);
		void convertK4aImageToOpenCvImage(const k4a_image_t &in, const int format, cv::Mat &out);
		void convertOpenCvImageToK4aImage(const cv::Mat &in, const int &format, k4a_image_t &out);

		#endif
		k4a::Vector extract_gravity_from_imu(k4a::Vector& imuAcc);
		string getSerialNumber(k4a_device_t device);
		string getSerialNumber();
		string getHardwareVersion();
		void getCapTemperature(float &temperature);
		void getImuTemperature(float &temperature);
		void getIMU(k4a::Vector& vec);
		void getIMU(float &x, float &y, float &z);
		void getIMUinDeg(float &x, float &y, float &z);
		void getGravity(k4a::Vector& vec);
		void getGravity(float &x, float &y, float &z);
		void getXYZrotationValuesInDeg(float &x, float &y, float &z);
		#ifdef USE_CV
		void getIntrinsicIr(cv::Mat &camMatrix, cv::Mat &coeff);
		void getIntrinsicColor(cv::Mat &camMatrix, cv::Mat &coeff);
		void getPrincipalPointColor(Point &po);
		void getPrincipalPoint2fColor(Point2f &po);
		void getPrincipalPointIr(Point &po);
		void getPrincipalPoint2fIr(Point2f &po);		
		void drawPrincipalPointColor(const cv::Mat &in, cv::Mat &out);
		void drawPrincipalPointIR(const cv::Mat &in, cv::Mat &out);
		int detectCurrentImageBlurStatusViaLaplacian(const cv::Mat &img);
		int detectCurrentImageBlurStatusViaFFT(const cv::Mat &img, const double threshold);
		void getCvColorImg(cv::Mat &img);
		void getCvIrImg(cv::Mat &img);
		void getCvDepthImg(cv::Mat &img);
		void getCvRegDepthImg(cv::Mat &img);
		void calcCvDepthGradientImg(const cv::Mat &img, cv::Mat &imgOut);
		void cropKv3Img(cv::Mat &imColor, cv::Mat &imDepth, const int w, const int h);		
		void resizeKv3Img(cv::Mat &imColor, cv::Mat &imDepth, const int inW, const int inH); //size sync for PCL creating
		void convIrToVisibleImg(const cv::Mat &imgIr, cv::Mat &dest);
		void convIrToVisible8BitGrayImg(const cv::Mat &src, cv::Mat &dest);
		void convToVisibleImg(const cv::Mat &in, cv::Mat &out);

		void cropKv3IrImg(cv::Mat &imIr, const int w, const int h);		
		void resizeKv3IrImg(cv::Mat &imIr, const int inW, const int inH);
		void resizeKv3ImgColor(cv::Mat &imColor, const int inW, const int inH);
		void resizeImg(cv::Mat &img, const int inW, const int inH);

		void calcCvPclCamColor(const cv::Mat &depth, cv::Mat &cvPcl);
		void calcCvPclCamResize(const cv::Mat &depth, cv::Mat &cvPcl, const int tarW, const int tarH, const bool bColor);
		void calcCvPclCamIr(const cv::Mat &depth, cv::Mat &cvPcl);
		void calcCvPclSimple(const cv::Mat &depth, cv::Mat &cvPcl);
		void calcAngleBetween3DvectorPoints(const Point3f &p1, const Point3f &p2, double &outAngle);
		void calcDeltaAngleFromPrincipalPointTofixPoint(const Point3d &poFix, float &ax, float &ay);
		void saveKv3Images(string path, long int counter, kv3 &kv3AppObj, int widthOfPcl, Mat &resizeImgColor, Mat &resizeImgColorDepth, Mat &regDepth, Mat &colorImg);
		void saveColorImg(string path, long int counter, Mat &colorImg);
		void saveRGBDImg(string path, long int counter, Mat &RGB, Mat&D);
		void savekv3Dataset(string path, long int counter, Mat &RGB, Mat&D, kv3 kv3AppObj);
#endif
		void savekv3PclDirect(string path);
		void savekv3PclDirectAsPcdfile(string path);
		void savekv3data(string path, long int counter);
		void savekv3gravitydata(string path, long int counter);
		static int imgWidth;
		static int imgHeight;
		static k4a_device_t device;
		k4a_hardware_version_t *k4aHwVer_HwVersion;
		k4a_calibration_t calibration;
		static k4a_capture_t capture;
		static k4a_transformation_t transformation;
		const int32_t TIMEOUT_IN_MS = 1000;
		static k4a_image_t kv3ImColor;
	    static k4a_image_t kv3ImColorRegDepth;
		static k4a_image_t kv3ImDepth;
		static k4a_image_t kv3ImIr;
		//k4a_image_t kv3ImDepthReg;
		k4a_imu_sample_t imu_sample;
		k4a::Vector imuAcc;
		k4a::Vector imuGravity;

		//PLC
		//static k4a_image_t kv3_depth_image;
    	static k4a_image_t kv3_xy_table;
    	static k4a_image_t kv3ImPcl;
		static k4a_image_t kv3ImPclReg;
    	static int kv3_point_count;

		static string serialNumber;
		static float imuTemperature;
		static float capTemparature;
 #ifdef USE_CV       
		static cv::Mat camMatrixKv3Ir;
		static cv::Mat distCoeffsKv3Ir;		
		static cv::Mat camMatrixKv3Color;
		static cv::Mat distCoeffsKv3Color; 
        cv::Mat imgColor;
		cv::Mat imgColorRegDepth;		
        cv::Mat imgDepth;
		cv::Mat imgIr;
        
#endif
		static float cx0;// depth camera principal point x
		static float cy0;// depth camera principal point y
		static float fx0;// depth camera focal length x
		static float fy0;// depth camera focal length y

		static float cx1;// color camera principal point x
		static float cy1;// color camera principal point y
		static float fx1;// color camera focal length x
		static float fy1;// color camera focal length y

		private:
		static k4a_device_configuration_t config;
		static int currentSetupId;
		static int depthFieldWidth;
		static int depthFieldHeight;
		static int rgbWidth;
		static int rgbHeight;

		static int imageMJPEGFormat;
		static int fpsMode;
};

#endif /* KV3_HPP_ */
