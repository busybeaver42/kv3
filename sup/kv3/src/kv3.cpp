/**
 * @file  kv3.cpp
 * @brief Implementation of the class functions to access the kinect azure sensor set data.
 * @brief created on:  09.02.2023;  last update: 24.06.2023
 * @author Jörg Angermayer\n
 * @copyright Licensed under MIT
 */
// thx for the doxygen tutorial to: https://darkognu.eu/programming/tutorials/doxygen_tutorial_cpp/

#include "kv3.hpp"

k4a_device_configuration_t kv3::config;

int kv3::imageMJPEGFormat = 0;
int kv3::fpsMode = 1; //1 = 15 FPS

int kv3::imgWidth = 360;//640;
int kv3::imgHeight = 360;//480;

k4a_device_t kv3::device = NULL;
k4a_capture_t kv3::capture = NULL;
k4a_transformation_t kv3::transformation = NULL;


k4a_image_t kv3::kv3ImColor = NULL;
k4a_image_t kv3::kv3ImColorRegDepth = NULL;
k4a_image_t kv3::kv3ImDepth = NULL;
k4a_image_t kv3::kv3ImIr = NULL;

int kv3::currentSetupId = 1;
string kv3::serialNumber;
float kv3::imuTemperature;
float kv3::capTemparature;

float kv3::cx0 = 0;
float kv3::cy0 = 0;
float kv3::fx0 = 0;
float kv3::fy0 = 0;
float kv3::cx1 = 0;
float kv3::cy1 = 0;
float kv3::fx1 = 0;
float kv3::fy1 = 0;

#ifdef USE_CV
//IR
cv::Mat kv3::camMatrixKv3Ir = (Mat_<double>(3,3) << 317.151, 0.0, 504.377, 0.0, 504.492, 334.9, 0.0, 0.0, 1.0);// 640x576
    //IR 640x576
    //cv::Mat kv3::camMatrixKv3Ir = (Mat_<double>(3,3) << 317.151, 0.0, 504.377, 0.0, 504.492, 334.9, 0.0, 0.0, 1.0);
    //IR 320x288
    //cv::Mat kv3::camMatrixKv3Ir = (Mat_<double>(3,3) << 158.326, 0.0, 252.189, 0.0, 252.246, 167.2, 0.0, 0.0, 1.0);
    //IR 1024x1024
    //cv::Mat kv3::camMatrixKv3Ir = (Mat_<double>(3,3) << 509.151, 0.0, 504.377, 0.0, 504.492, 514.9, 0.0, 0.0, 1.0);

		
//Color
cv::Mat kv3::camMatrixKv3Color = (Mat_<double>(3,3) << 604.696, 0.0, 645.99, 0.0, 604.769, 367.961, 0.0, 0.0, 1.0);// 1280x720
    //Color 1280x720
    //cv::Mat kv3::camMatrixKv3Color = (Mat_<double>(3,3) << 604.696, 0.0, 645.99, 0.0, 604.769, 367.961, 0.0, 0.0, 1.0);
    //Color 1920x1080
    //cv::Mat kv3::camMatrixKv3Color = (Mat_<double>(3,3) << 969.235, 0.0, 907.044, 0.0, 907.154, 552.192, 0.0, 0.0, 1.0);
    //Color 4096x3072
    //cv::Mat kv3::camMatrixKv3Color = (Mat_<double>(3,3) << 2068.27, 0.0, 1935.03, 0.0, 1935.26, 1562.58, 0.0, 0.0, 1.0);

cv::Mat kv3::distCoeffsKv3Ir = (Mat_<double>(1,8) << 0.653048, 0.234488, 4.05565e-05, -0.000114827, 0.0125106, 0.990673, 0.391103, 0.06483);
cv::Mat kv3::distCoeffsKv3Color = (Mat_<double>(1,8) << 0.36451, -2.55157, 3.19741e-05, -0.000113713, 1.49545, 0.244343, -2.36779, 1.41759);
#endif
//PLC
//k4a_image_t kv3::kv3_depth_image = NULL;
k4a_image_t kv3::kv3_xy_table = NULL;
k4a_image_t kv3::kv3ImPclReg = NULL;
k4a_image_t kv3::kv3ImPcl = NULL;



int kv3::kv3_point_count = 0;
int kv3::depthFieldWidth;
int kv3::depthFieldHeight;
int kv3::rgbWidth;
int kv3::rgbHeight;


kv3::kv3() {
   // k4a_set_debug_message_handler(nullptr, nullptr, K4A_LOG_LEVEL_OFF);
   // std::system("export K4A_ENABLE_LOG_TO_STDOUT=0");
   
}
kv3::~kv3() {
    shutdown();
    releaseCapture();
}

 /**
  *     This function release all open captured k4a_image_t images
  *     - kv3ImColor
  *     - kv3ImColorRegDepth
  *     - kv3ImDepth
  *     - kv3ImIr.
  * \param	void parameter  
  * \return	no return value
  *
  */
void kv3::shutdown(){
    if(kv3ImColor != NULL){
        k4a_image_release(kv3ImColor);
    }
    if(kv3ImDepth != NULL){
        k4a_image_release(kv3ImDepth);
    }
    if(kv3ImColorRegDepth != NULL){
        k4a_image_release(kv3ImColorRegDepth);
    }    
    if(kv3ImIr != NULL){
        k4a_image_release(kv3ImIr);
    }        
    if (device != NULL){
        k4a_device_stop_cameras(device);
        k4a_device_close(device);
    }      
}

 /**
  *     This function release open capture handle
  * \param	void parameter  
  * \return	no return value
  *
  */
void kv3::releaseCapture(){
    if (capture != NULL){
        k4a_capture_release(capture);
    }  
}

 /**
  * \brief inline string kv3::getSerialNumber(k4a_device_t device).
  *
  *     This function read the serial number from kinect azure sensor device.

  * \param	input: k4a_device_t device handle from kinect azure device .
  * \return	std::string with the serial number.
  *
  */
inline string kv3::getSerialNumber(k4a_device_t device)
{
    size_t serial_number_length = 0;
    
    if (K4A_BUFFER_RESULT_TOO_SMALL != k4a_device_get_serialnum(device, NULL, &serial_number_length))
    {
        std::cout << "Failed to get serial number length" << endl;
        k4a_device_close(device);
        exit(-1);
    }

    char *serial_number = new (std::nothrow) char[serial_number_length];
    if (serial_number == NULL)
    {
        std::cout << "Failed to allocate memory for serial number (" << serial_number_length << " bytes)" << endl;
        k4a_device_close(device);
        exit(-1);
    }

    if (K4A_BUFFER_RESULT_SUCCEEDED != k4a_device_get_serialnum(device, serial_number, &serial_number_length))
    {
        std::cout << "Failed to get serial number" << endl;
        delete[] serial_number;
        serial_number = NULL;
        k4a_device_close(device);
        exit(-1);
    }

    string s(serial_number);
    delete[] serial_number;
    serial_number = NULL;
    kv3::serialNumber = s;
    return s;
}

 /**
  * Deliver the serial number from device (MS kinect azure)
  * \param	void parameter  
  * \return	serial number from device as std::string
  *
  */
string kv3::getSerialNumber(){
    return(kv3::serialNumber);
}


 /**
  * create values to improve the depth values
  * input:\n
  * \param	const k4a_calibration_t *calibration = internal calibration data  
  * output:\n
  * \param	k4a_image_t xy_table = values to improve the depth values
  * \return	void
  *
  */
void kv3::create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table)
{
    k4a_float2_t *table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;

    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;

    for (int y = 0, idx = 0; y < height; y++)
    {
        p.xy.y = (float)y;
        for (int x = 0; x < width; x++, idx++)
        {
            p.xy.x = (float)x;

            k4a_calibration_2d_to_3d(
                calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

            if (valid)
            {
                table_data[idx].xy.x = ray.xyz.x;
                table_data[idx].xy.y = ray.xyz.y;
            }
            else
            {
                table_data[idx].xy.x = nanf("");
                table_data[idx].xy.y = nanf("");
            }
        }
    }
}

 /**
  * setup the camera calibration from the depth camera and the color camera.\n
  * input:\n
  * \param	void
  * \return	void
  */
inline void kv3::setupCalib(){
    k4a_calibration_camera_t calib0 = calibration.depth_camera_calibration;

    std::cout << "\n===== Device "  << ": " << getSerialNumber(device) << " =====\n";
    std::cout << "depth camera resolution width: " << calib0.resolution_width << endl;
    std::cout << "depth camera resolution height: " << calib0.resolution_height << endl;
    std::cout << "depth camera principal point x: " << calib0.intrinsics.parameters.param.cx << endl;
    std::cout << "depth camera principal point y: " << calib0.intrinsics.parameters.param.cy << endl;
    std::cout << "depth camera focal length x: " << calib0.intrinsics.parameters.param.fx << endl;
    std::cout << "depth camera focal length y: " << calib0.intrinsics.parameters.param.fy << endl;
    std::cout << "depth camera radial distortion coefficients:" << endl;
    std::cout << "depth camera k1: " << calib0.intrinsics.parameters.param.k1 << endl;
    std::cout << "depth camera k2: " << calib0.intrinsics.parameters.param.k2 << endl;
    std::cout << "depth camera k3: " << calib0.intrinsics.parameters.param.k3 << endl;
    std::cout << "depth camera k4: " << calib0.intrinsics.parameters.param.k4 << endl;
    std::cout << "depth camera k5: " << calib0.intrinsics.parameters.param.k5 << endl;
    std::cout << "depth camera k6: " << calib0.intrinsics.parameters.param.k6 << endl;
    std::cout << "depth camera center of distortion in Z=1 plane, x: " << calib0.intrinsics.parameters.param.codx << endl;
    std::cout << "depth camera center of distortion in Z=1 plane, y: " << calib0.intrinsics.parameters.param.cody << endl;
    std::cout << "depth camera tangential distortion coefficient x: " << calib0.intrinsics.parameters.param.p1 << endl;
    std::cout << "depth camera tangential distortion coefficient y: " << calib0.intrinsics.parameters.param.p2 << endl;
    std::cout << "depth camera metric radius: " << calib0.intrinsics.parameters.param.metric_radius << endl;

    k4a_calibration_camera_t calib1 = calibration.color_camera_calibration;
    std::cout << "color camera resolution width: " << calib1.resolution_width << endl;
    std::cout << "color camera resolution height: " << calib1.resolution_height << endl;
    std::cout << "color camera principal point x: " << calib1.intrinsics.parameters.param.cx << endl;
    std::cout << "color camera principal point y: " << calib1.intrinsics.parameters.param.cy << endl;
    std::cout << "color camera focal length x: " << calib1.intrinsics.parameters.param.fx << endl;
    std::cout << "color camera focal length y: " << calib1.intrinsics.parameters.param.fy << endl;
    std::cout << "color camera radial distortion coefficients:" << endl;
    std::cout << "color camera k1: " << calib1.intrinsics.parameters.param.k1 << endl;

    std::cout << "color camera k2: " << calib1.intrinsics.parameters.param.k2 << endl;
    std::cout << "color camera k3: " << calib1.intrinsics.parameters.param.k3 << endl;
    std::cout << "color camera k4: " << calib1.intrinsics.parameters.param.k4 << endl;
    std::cout << "color camera k5: " << calib1.intrinsics.parameters.param.k5 << endl;
    std::cout << "color camera k6: " << calib1.intrinsics.parameters.param.k6 << endl;

    double colork1 = (double)calib1.intrinsics.parameters.param.k1;
    std::cout << "colork1: " << colork1 << endl;

    std::cout << "color camera center of distortion in Z=1 plane, x: " << calib1.intrinsics.parameters.param.codx << endl;
    std::cout << "color camera center of distortion in Z=1 plane, y: " << calib1.intrinsics.parameters.param.cody << endl;
    std::cout << "color camera tangential distortion coefficient x: " << calib1.intrinsics.parameters.param.p1 << endl;
    std::cout << "color camera tangential distortion coefficient y: " << calib1.intrinsics.parameters.param.p2 << endl;
    std::cout << "color camera metric radius: " << calib1.intrinsics.parameters.param.metric_radius << endl;

    transformation = k4a_transformation_create(&calibration);

    kv3::cx0 = (float)calib0.intrinsics.parameters.param.cx;
    kv3::cy0 = (float)calib0.intrinsics.parameters.param.cy;
    kv3::fx0 = (float)calib0.intrinsics.parameters.param.fx;
    kv3::fy0 = (float)calib0.intrinsics.parameters.param.fy;

    kv3::cx1 = (float)calib1.intrinsics.parameters.param.cx;
    kv3::cy1 = (float)calib1.intrinsics.parameters.param.cy;
    kv3::fx1 = (float)calib1.intrinsics.parameters.param.fx;
    kv3::fy1 = (float)calib1.intrinsics.parameters.param.fy;
    //cout << "--------------------" << endl;
    #ifdef USE_CV
    //update intrinsic data IR camera Matrix
     //cv::Mat kv3::camMatrixKv3Ir = (Mat_<double>(3,3) << 317.151, 0.0, 504.377, 0.0, 504.492, 334.9, 0.0, 0.0, 1.0);// 640x576
    kv3::camMatrixKv3Ir.at<double>(0,0) = (double)calib0.intrinsics.parameters.param.fx;
    kv3::camMatrixKv3Ir.at<double>(0,1) = (double)0;
    kv3::camMatrixKv3Ir.at<double>(0,2) = (double)calib0.intrinsics.parameters.param.cx;
    kv3::camMatrixKv3Ir.at<double>(1,0) = (double)0;
    kv3::camMatrixKv3Ir.at<double>(1,1) = (double)calib0.intrinsics.parameters.param.fy;
    kv3::camMatrixKv3Ir.at<double>(1,2) = (double)calib0.intrinsics.parameters.param.cy;
    kv3::camMatrixKv3Ir.at<double>(2,0) = (double)0;
    kv3::camMatrixKv3Ir.at<double>(2,1) = (double)0;
    kv3::camMatrixKv3Ir.at<double>(2,2) = (double)1;    
    //cout << "camMatrixKv3Ir: " << camMatrixKv3Color << endl;
    //cout << "--------------------" << endl;
     //update intrinsic data Color camera Matrix
     //cv::Mat kv3::camMatrixKv3Ir = (Mat_<double>(3,3) << 317.151, 0.0, 504.377, 0.0, 504.492, 334.9, 0.0, 0.0, 1.0);// 640x576
    kv3::camMatrixKv3Color.at<double>(0,0) = (double)calib1.intrinsics.parameters.param.fx;
    kv3::camMatrixKv3Color.at<double>(0,1) = (double)0;
    kv3::camMatrixKv3Color.at<double>(0,2) = (double)calib1.intrinsics.parameters.param.cx;
    kv3::camMatrixKv3Color.at<double>(1,0) = (double)0;
    kv3::camMatrixKv3Color.at<double>(1,1) = (double)calib1.intrinsics.parameters.param.fy;
    kv3::camMatrixKv3Color.at<double>(1,2) = (double)calib1.intrinsics.parameters.param.cy;
    kv3::camMatrixKv3Color.at<double>(2,0) = (double)0;
    kv3::camMatrixKv3Color.at<double>(2,1) = (double)0;
    kv3::camMatrixKv3Color.at<double>(2,2) = (double)1;
    //cout << "--------------------" << endl;
    //cout << "camMatrixKv3Color: " << camMatrixKv3Color << endl;
    //update intrinsic data IR distortion Coeff.
    kv3::distCoeffsKv3Ir.at<double>(0) = colork1;
    kv3::distCoeffsKv3Ir.at<double>(1) = (double)calib0.intrinsics.parameters.param.k2;
    kv3::distCoeffsKv3Ir.at<double>(2) = (double)calib0.intrinsics.parameters.param.p1;
    kv3::distCoeffsKv3Ir.at<double>(3) = (double)calib0.intrinsics.parameters.param.p2;
    kv3::distCoeffsKv3Ir.at<double>(4) = (double)calib0.intrinsics.parameters.param.k3;
    kv3::distCoeffsKv3Ir.at<double>(5) = (double)calib0.intrinsics.parameters.param.k4;
    kv3::distCoeffsKv3Ir.at<double>(6) = (double)calib0.intrinsics.parameters.param.k5;
    kv3::distCoeffsKv3Ir.at<double>(7) = (double)calib0.intrinsics.parameters.param.k6;
    //cout << "--------------------" << endl;
    //cout << "distCoeffsKv3Ir: " << distCoeffsKv3Color << endl;
    //update intrinsic data color distortion Coeff.
    kv3::distCoeffsKv3Color.at<double>(0) = (double)calib1.intrinsics.parameters.param.k1;
    kv3::distCoeffsKv3Color.at<double>(1) = (double)calib1.intrinsics.parameters.param.k2;
    kv3::distCoeffsKv3Color.at<double>(2) = (double)calib1.intrinsics.parameters.param.p1;
    kv3::distCoeffsKv3Color.at<double>(3) = (double)calib1.intrinsics.parameters.param.p2;
    kv3::distCoeffsKv3Color.at<double>(4) = (double)calib1.intrinsics.parameters.param.k3;
    kv3::distCoeffsKv3Color.at<double>(5) = (double)calib1.intrinsics.parameters.param.k4;
    kv3::distCoeffsKv3Color.at<double>(6) = (double)calib1.intrinsics.parameters.param.k5;
    kv3::distCoeffsKv3Color.at<double>(7) = (double)calib1.intrinsics.parameters.param.k6;
    //cout << "--------------------" << endl;
    //cout << "distCoeffsKv3Color: " << distCoeffsKv3Color << endl;
    std::cout << "--------------------" << endl;
    std::cout << "--------------------" << endl;
    #endif
    //cv::Mat kv3::distCoeffsKv3Ir = (Mat_<double>(1,8) << 0.653048, 0.234488, 4.05565e-05, -0.000114827, 0.0125106, 0.990673, 0.391103, 0.06483);
    //cv::Mat kv3::distCoeffsKv3Color = (Mat_<double>(1,8) << 0.36451, -2.55157, 3.19741e-05, -0.000113713, 1.49545, 0.244343, -2.36779, 1.41759);
    
}

 /**
  * read the current color mode from the MS Kinect Azure.\n
  * input:\n
  * \param	void
  * Output:\n
  * \param	no, but output into the console. 
  * \return	void
  */
void kv3::readCurrentColorMode(){
/*
k4a_result_t k4a_device_get_color_control 	( 	k4a_device_t  	device_handle,
		k4a_color_control_command_t  	command,
		k4a_color_control_mode_t *  	mode,
		int32_t *  	value 
	) 	
*/
    k4a_color_control_command_t  	command;
    k4a_color_control_mode_t *  	mode;
    int32_t *  	value ;

    k4a_color_control_mode_t  	myMode;
    command = K4A_COLOR_CONTROL_POWERLINE_FREQUENCY;
    //command = K4A_COLOR_CONTROL_BRIGHTNESS;
    //command = K4A_COLOR_CONTROL_WHITEBALANCE; //available in auto mode
    //command = K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE; //available in auto mode

    if (K4A_RESULT_SUCCEEDED != k4a_device_get_color_control(device, command, mode, value)){
        std::cout << "---- ERROR --- read k4a_device_get_color_control was not succesfull" << endl;
        //https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/940
        std::cout << "---- Automatic control is active, if the firmware < AzureKinectDK_Fw_1.6.110079014.bin" << endl;
    }else{
        
        myMode = *mode;
        if(myMode == K4A_COLOR_CONTROL_MODE_AUTO){
            std::cout << "auto is active.";
        }
        if(myMode == K4A_COLOR_CONTROL_MODE_MANUAL){
            std::cout << "manual is active.";
            /*
            int32_t myValue = *value;
            if(myValue == 1){
                cout << "powerline compensation for 50 Hz is active.";
            }
            
            if(myValue== 2){
                cout << "powerline compensation for 60 Hz is active.";
            }        
            */    
        }
        
    }
}


 /**
  * setup the MS Kinect Azure image format and FPS rate\n
  * input:\n
  * \param	void
  * Output:\n
  * \param	void
  * \return	void
  */
void kv3::setupImFormatAndFPSconfig(){
    kv3::config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

    //image format
    if (kv3::imageMJPEGFormat == 0){
        kv3::config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;  
    }    
    if (kv3::imageMJPEGFormat == 1){
        kv3::config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG; 
    }

    //FPS
    if(kv3::fpsMode == 0){
        config.camera_fps = K4A_FRAMES_PER_SECOND_5;
    }
    if(kv3::fpsMode == 1){
        config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    }
    if(kv3::fpsMode == 2){
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    }         
}
 /**
  * setup the MS Kinect Azure modes\n
  * input:\n
  * \param	void
  * Output:\n
  * \param	void
  * \return	void
  */
void kv3::setup01(){
    if(this->device != NULL){
        k4a_device_close(this->device);            
    }    
    uint32_t device_count = k4a_device_get_installed_count();
    if (device_count == 0)
    {
        std::cout << "No K4A devices found" << endl;
        exit(0);
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        std::cout << "No K4A devices found" << endl;
        if (device != NULL)
        {
            k4a_device_close(device);
        }
    }

    /*
    Config from - KinectAzureCapture --- https://github.com/WeLoveKiraboshi/KinectAzureCapture
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.synchronized_images_only = true;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;//examples/transformation
    config.color_resolution = K4A_COLOR_RESOLUTION_1536P; //K4A_COLOR_RESOLUTION_720P;
    //config.depth_format = K4A_IMAGE_FORMAT_DEPTH16;//https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/

    //config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;//640x576.
    config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;//1024x1024.
    config.camera_fps = K4A_FRAMES_PER_SECOND_15; //30
    */
    /*
    Config from - k4a-measurement
    // Set the configuration of device, you can also set it after open the device but before starting the camera
    */
    //k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

    setFpsMode(1);
    setupImFormatAndFPSconfig();
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;//K4A_COLOR_RESOLUTION_720P;
    rgbWidth = 1920;
    rgbHeight = 1080;    
    config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED; //1024x1024 ; 0.25 - 2.21 m
    depthFieldWidth = 1024;
	depthFieldHeight = 1024;

    config.synchronized_images_only = true;
   
    /*
    Depth: config.depth_mode

        K4A_DEPTH_MODE_OFF 	
        Depth sensor will be turned off with this setting.

        K4A_DEPTH_MODE_NFOV_2X2BINNED 	
        Depth captured at 320x288.
        Passive IR is also captured at 320x288.

        K4A_DEPTH_MODE_NFOV_UNBINNED 	
        Depth captured at 640x576.
        Passive IR is also captured at 640x576.

        K4A_DEPTH_MODE_WFOV_2X2BINNED 	
        Depth captured at 512x512.
        Passive IR is also captured at 512x512.

        K4A_DEPTH_MODE_WFOV_UNBINNED 	
        Depth captured at 1024x1024.
        Passive IR is also captured at 1024x1024.

        K4A_DEPTH_MODE_PASSIVE_IR 	
        Passive IR only, captured at 1024x1024. 


    COLOR: config.color_resolution
        K4A_COLOR_RESOLUTION_OFF 	
        Color camera will be turned off with this setting.

        K4A_COLOR_RESOLUTION_720P 	
        1280 * 720 16:9

        K4A_COLOR_RESOLUTION_1080P 	
        1920 * 1080 16:9

        K4A_COLOR_RESOLUTION_1440P 	
        2560 * 1440 16:9

        K4A_COLOR_RESOLUTION_1536P 	
        2048 * 1536 4:3

        K4A_COLOR_RESOLUTION_2160P 	
        3840 * 2160 16:9

        K4A_COLOR_RESOLUTION_3072P 	
        4096 * 3072 4:3         
   */

    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        std::cout << "Failed to get calibration" << endl;
        exit(0);
    }

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                        calibration.depth_camera_calibration.resolution_width,
                        calibration.depth_camera_calibration.resolution_height,
                        calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
                        &kv3_xy_table);

        create_xy_table(&calibration, kv3_xy_table);

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                        calibration.depth_camera_calibration.resolution_width,
                        calibration.depth_camera_calibration.resolution_height,
                        calibration.depth_camera_calibration.resolution_width * 3 * (int)sizeof(int16_t),
                        &kv3ImPcl);    

    setupCalib(); //get intrinsic data from color and depth cam

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        std::cout << "Failed to start cameras" << endl;
        exit(0);
    }
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_imu(device))
    {
        std::cout <<"Failed to get imu data from capture" << endl;
        exit(0);
    }
    readCurrentColorMode();
    currentSetupId = 1;
}


 /**
  * setup 02 - the MS Kinect Azure modes\n
  * input:\n
  * \param	void
  * Output:\n
  * \param	void
  * \return	void
  */
void kv3::setup02(){
    if(this->device != NULL){
        k4a_device_close(this->device);            
    }

    uint32_t device_count = k4a_device_get_installed_count();
    if (device_count == 0)
    {
        std::cout << "No K4A devices found" << endl;
        exit(0);
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        std::cout << "No K4A devices found" << endl;
        if (device != NULL)
        {
            k4a_device_close(device);
        }
    }
    setFpsMode(0);
    setupImFormatAndFPSconfig();
    config.color_resolution = K4A_COLOR_RESOLUTION_3072P; 
    rgbWidth = 4096;
    rgbHeight = 3072;    
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    depthFieldWidth = 640;
	depthFieldHeight = 576;

    config.synchronized_images_only = true;
   
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        std::cout << "Failed to get calibration" << endl;
        exit(0);
    }

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                        calibration.depth_camera_calibration.resolution_width,
                        calibration.depth_camera_calibration.resolution_height,
                        calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
                        &kv3_xy_table);

        create_xy_table(&calibration, kv3_xy_table);

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                        calibration.depth_camera_calibration.resolution_width,
                        calibration.depth_camera_calibration.resolution_height,
                        calibration.depth_camera_calibration.resolution_width * 3 * (int)sizeof(int16_t),
                        &kv3ImPcl);    

    setupCalib(); //get intrinsic data from color and depth cam

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        std::cout << "Failed to start cameras" << endl;
        exit(0);
    }
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_imu(device))
    {
        std::cout <<"Failed to get imu data from capture" << endl;
        exit(0);
    }
    readCurrentColorMode();
    currentSetupId = 2;
}


 /**
  * setup 03 the MS Kinect Azure modes\n
  * input:\n
  * \param	void
  * Output:\n
  * \param	void
  * \return	void
  */
void kv3::setup03(){
    if(this->device != NULL){
        k4a_device_close(this->device);            
    }
    uint32_t device_count = k4a_device_get_installed_count();
    if (device_count == 0)
    {
        std::cout << "No K4A devices found" << endl;
        exit(0);
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        std::cout << "No K4A devices found" << endl;
        if (device != NULL)
        {
            k4a_device_close(device);
        }
    }
    setFpsMode(2);
    setupImFormatAndFPSconfig();
    config.color_resolution = K4A_COLOR_RESOLUTION_720P; //K4A_COLOR_RESOLUTION_1080P;//K4A_COLOR_RESOLUTION_2160P;//K4A_COLOR_RESOLUTION_3072P;
    rgbWidth = 1280;
    rgbHeight = 720;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED ;
    depthFieldWidth = 640;
	depthFieldHeight = 576;

    config.synchronized_images_only = true;

    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        std::cout << "Failed to get calibration" << endl;
        exit(0);
    }

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                        calibration.depth_camera_calibration.resolution_width,
                        calibration.depth_camera_calibration.resolution_height,
                        calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
                        &kv3_xy_table);

        create_xy_table(&calibration, kv3_xy_table);

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                        calibration.depth_camera_calibration.resolution_width,
                        calibration.depth_camera_calibration.resolution_height,
                        calibration.depth_camera_calibration.resolution_width * 3 * (int)sizeof(int16_t),
                        &kv3ImPcl);    

    setupCalib(); //get intrinsic data from color and depth cam

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        std::cout << "Failed to start cameras" << endl;
        exit(0);
    }
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_imu(device))
    {
        std::cout <<"Failed to get imu data from capture" << endl;
        exit(0);
    }
    readCurrentColorMode();
    currentSetupId = 3;
}


 /**
  * setup 04 the MS Kinect Azure modes\n
  * input:\n
  * \param	void
  * Output:\n
  * \param	void
  * \return	void
  */
void kv3::setup04(){
    if(this->device != NULL){
        k4a_device_close(this->device);            
    }
    uint32_t device_count = k4a_device_get_installed_count();
    if (device_count == 0)
    {
        std::cout << "No K4A devices found" << endl;
        exit(0);
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        std::cout << "No K4A devices found" << endl;
        if (device != NULL)
        {
            k4a_device_close(device);
        }
    }

    setFpsMode(2);
    setupImFormatAndFPSconfig();
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    rgbWidth = 1280;
    rgbHeight = 720;    
    config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED; //320x288 ; 0.5 - 5.46 m
    depthFieldWidth = 320;
	depthFieldHeight = 288;

    config.synchronized_images_only = true;

    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        std::cout << "Failed to get calibration" << endl;
        exit(0);
    }

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                        calibration.depth_camera_calibration.resolution_width,
                        calibration.depth_camera_calibration.resolution_height,
                        calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
                        &kv3_xy_table);

        create_xy_table(&calibration, kv3_xy_table);

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                        calibration.depth_camera_calibration.resolution_width,
                        calibration.depth_camera_calibration.resolution_height,
                        calibration.depth_camera_calibration.resolution_width * 3 * (int)sizeof(int16_t),
                        &kv3ImPcl);    

    setupCalib(); //get intrinsic data from color and depth cam

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        std::cout << "Failed to start cameras" << endl;
        exit(0);
    }
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_imu(device))
    {
        std::cout <<"Failed to get imu data from capture" << endl;
        exit(0);
    }
    readCurrentColorMode();
    currentSetupId = 4;
}


/**
  * setup 05 the MS Kinect Azure modes\n
  * input:\n
  * \param	void
  * Output:\n
  * \param	void
  * \return	void
  */
void kv3::setup05(){
    if(this->device != NULL){
        k4a_device_close(this->device);            
    }

    uint32_t device_count = k4a_device_get_installed_count();
    if (device_count == 0)
    {
        std::cout << "No K4A devices found" << endl;
        exit(0);
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        std::cout << "No K4A devices found" << endl;
        if (device != NULL)
        {
            k4a_device_close(device);
        }
    }
    setFpsMode(0);
    setupImFormatAndFPSconfig();
    config.color_resolution = K4A_COLOR_RESOLUTION_3072P; 
    rgbWidth = 4096;
    rgbHeight = 3072;    
    config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;//K4A_DEPTH_MODE_NFOV_UNBINNED;
    depthFieldWidth = 1024;
	depthFieldHeight = 1024;

    config.synchronized_images_only = true;
   
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        std::cout << "Failed to get calibration" << endl;
        exit(0);
    }

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                        calibration.depth_camera_calibration.resolution_width,
                        calibration.depth_camera_calibration.resolution_height,
                        calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
                        &kv3_xy_table);

        create_xy_table(&calibration, kv3_xy_table);

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                        calibration.depth_camera_calibration.resolution_width,
                        calibration.depth_camera_calibration.resolution_height,
                        calibration.depth_camera_calibration.resolution_width * 3 * (int)sizeof(int16_t),
                        &kv3ImPcl);

    setupCalib(); //get intrinsic data from color and depth cam

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        std::cout << "Failed to start cameras" << endl;
        exit(0);
    }
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_imu(device))
    {
        std::cout <<"Failed to get imu data from capture" << endl;
        exit(0);
    }
    readCurrentColorMode();
    currentSetupId = 5;
}

/**
  * setup 06 the MS Kinect Azure modes\n
  * input:\n
  * \param	void
  * Output:\n
  * \param	void
  * \return	void
  */
void kv3::setup06(){
    if(this->device != NULL){
        k4a_device_close(this->device);            
    }

    uint32_t device_count = k4a_device_get_installed_count();
    if (device_count == 0)
    {
        std::cout << "No K4A devices found" << endl;
        exit(0);
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        std::cout << "No K4A devices found" << endl;
        if (device != NULL)
        {
            k4a_device_close(device);
        }
    }

    setFpsMode(2);
    setupImFormatAndFPSconfig();
    config.color_resolution = K4A_COLOR_RESOLUTION_1536P; 
    rgbWidth = 2048;
    rgbHeight = 1536;    
    config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;//K4A_DEPTH_MODE_WFOV_UNBINNED;6
    depthFieldWidth = 512;
	depthFieldHeight = 512;

    config.synchronized_images_only = true;
   
    if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        std::cout << "Failed to get calibration" << endl;
        exit(0);
    }

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                        calibration.depth_camera_calibration.resolution_width,
                        calibration.depth_camera_calibration.resolution_height,
                        calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
                        &kv3_xy_table);

        create_xy_table(&calibration, kv3_xy_table);

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                        calibration.depth_camera_calibration.resolution_width,
                        calibration.depth_camera_calibration.resolution_height,
                        calibration.depth_camera_calibration.resolution_width * 3 * (int)sizeof(int16_t),
                        &kv3ImPcl);

    setupCalib(); //get intrinsic data from color and depth cam

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        std::cout << "Failed to start cameras" << endl;
        exit(0);
    }
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_imu(device))
    {
        std::cout <<"Failed to get imu data from capture" << endl;
        exit(0);
    }
    readCurrentColorMode();
    currentSetupId = 6;
}


/**
  * setup 07 the MS Kinect Azure modes\n
  * input:\n
  * \param	void
  * Output:\n
  * \param	void
  * \return	void
  */
void kv3::setup07(){
    if(this->device != NULL){
        k4a_device_close(this->device);            
    }

    uint32_t device_count = k4a_device_get_installed_count();
    if (device_count == 0)
    {
        std::cout << "No K4A devices found" << endl;
        exit(0);
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        std::cout << "No K4A devices found" << endl;
        if (device != NULL)
        {
            k4a_device_close(device);
        }
    }
    setFpsMode(0);
    setupImFormatAndFPSconfig();
    config.color_resolution = K4A_COLOR_RESOLUTION_3072P; 
    rgbWidth = 3072;
    rgbHeight = 4096;    
    config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;//K4A_DEPTH_MODE_WFOV_UNBINNED;;
    depthFieldWidth = 512;
	depthFieldHeight = 512;

    config.synchronized_images_only = true;
   
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        std::cout << "Failed to get calibration" << endl;
        exit(0);
    }

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                        calibration.depth_camera_calibration.resolution_width,
                        calibration.depth_camera_calibration.resolution_height,
                        calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
                        &kv3_xy_table);

        create_xy_table(&calibration, kv3_xy_table);

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                        calibration.depth_camera_calibration.resolution_width,
                        calibration.depth_camera_calibration.resolution_height,
                        calibration.depth_camera_calibration.resolution_width * 3 * (int)sizeof(int16_t),
                        &kv3ImPcl);

    setupCalib(); //get intrinsic data from color and depth cam

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        std::cout << "Failed to start cameras" << endl;
        exit(0);
    }
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_imu(device))
    {
        std::cout <<"Failed to get imu data from capture" << endl;
        exit(0);
    }
    readCurrentColorMode();
    currentSetupId = 7;
}

 /**
  * get the current rgb resolution defined by the current setup\n
  * input:\n
  * \param	void
  * Output:\n
  * \param	int x = the width of the depth, int y = the height of the depth 
  * \return	void
  */
void kv3::getRgbResolution(int &x, int &y){
    x = rgbWidth;
	y = rgbHeight;
}

 /**
  * get the current depth resolution defined by the current setup\n
  * input:\n
  * \param	void
  * Output:\n
  * \param	int x = the width of the depth, int y = the height of the depth 
  * \return	void
  */
void kv3::getDepthFieldResolution(int &x, int &y){
    x = depthFieldWidth;
	y = depthFieldHeight;
}

 /**
  * update all MS Kinect Azure sensor data (Ir, color, depth, IMU)\n
  * input:\n
  * \param	int mode := 0 = grab all picture, 1 = grab fast pipeline(depth, PCL and ColorReg)
  * Output:\n
  * \param	int retval 
  * \return	K4A_WAIT_RESULT_SUCCEEDED | K4A_WAIT_RESULT_SUCCEEDED | K4A_WAIT_RESULT_FAILED
  */
int kv3::update(int mode){
    int retval = 0;
    k4a_wait_result_t resultInfo = k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS);
       
    if(resultInfo == K4A_WAIT_RESULT_SUCCEEDED){
        #ifdef USE_CV
        if(mode == 0){
        capDepth();
        capPclDepth();        
            if (kv3::imageMJPEGFormat == 0){
                capColor();
                capColorReg();
            }    
            if (kv3::imageMJPEGFormat == 1){
                capColorMJPEG();
            }           
            capIr();
            updateIMU();
        }
        if(mode == 1){
            capDepth();
            capPclDepth(); 
            capColor();
            capColorReg();
        }

        #endif
        retval = (1);  
    }
    if(resultInfo == K4A_WAIT_RESULT_FAILED){
        std::cout << "Failed to read a capture" << endl;
        shutdown();
        retval = (-1);
    }     
    if(resultInfo == K4A_WAIT_RESULT_TIMEOUT){
        printf("Timed out waiting for a capture\n");
        retval = (2); 
    }
   
    if(resultInfo > 2){
        printf("unknown Error: %d \n", resultInfo);
        retval = (-1); 
    }


    return(retval);
}

 /**
  * release all MS Kinect Azure sensor image data (Ir, color, depth, ...)\n
  * input:\n
  * \param	void
  * Output:\n
  * \param	void
  * \return	void
  */
void kv3::releaseAllkv3img(){
    k4a_image_release(kv3ImDepth);
    k4a_image_release(kv3ImPcl);
    k4a_image_release(kv3ImColor);
    k4a_image_release(kv3ImColorRegDepth);
    k4a_image_release(kv3ImIr);
    k4a_capture_release(capture); 
}


 /**
  * Set image format for the color image\n
  * input: format 0=BRGA; 1=MJPEG\n
  * \param int 
  * Output:\n
  * \param	void
  * \return	void
  */
void kv3::setImageFormat(int format){
    if(format == 1){
        kv3::imageMJPEGFormat = 1;
    }else{
        kv3::imageMJPEGFormat = 0;
    }  
}

#ifdef USE_CV
 /**
  * demo to grap MS Kinect Azure data inside a loop (Ir, color, depth, IMU)\n
  * input:\n
  * \param	void
  * Output:\n
  * \param	void
  * \return	void
  */
void kv3::testloop(){
    k4a::Vector IMU_vec;
    k4a::Vector grav_vec;

    while(1){
        switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            if (kv3::imageMJPEGFormat == 0){
                capColor();
            }    
            if (kv3::imageMJPEGFormat == 1){
                capColorMJPEG();
            }
            capIr();
            capDepth();            
            capTemperature();        
            updateIMU();
            //getIMU(IMU_vec);
            //getGravity(grav_vec);
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for a capture\n");
            continue;
            break;
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a capture\n");
            if (device != NULL)
            {
                k4a_device_stop_cameras(device);
                k4a_device_close(device);
            }
        }

        char key = waitKey(10);
        if(key == 27){ break; }
    }

    // Shut down the camera when finished with application logic
    if(kv3ImColor != NULL){
        k4a_image_release(kv3ImColor);
    }
    if(kv3ImIr != NULL){
        k4a_image_release(kv3ImIr);
    }
    if(kv3ImDepth != NULL){
        k4a_image_release(kv3ImDepth);
    }
    if(kv3ImColorRegDepth != NULL){
        k4a_image_release(kv3ImColorRegDepth);
    }    
    if (device != NULL){
        k4a_device_stop_cameras(device);
        k4a_device_close(device);
    }    
}
#endif

#ifdef USE_CV
 /**
  * calculate via depth image very simple a point cloud\n
  * input:\n
  * \param	const cv::Mat &depth = opencv depth image
  * Output:\n
  * \param	cv::Mat &cvPcl = opencv point cloud (values are in meter)
  * \return	void
  */
void kv3::calcCvPclSimple(const cv::Mat &depth, cv::Mat &cvPcl){
    int xmax = depth.size().width;
    int ymax = depth.size().height;
    cv::Mat bufferPcl(ymax, xmax, CV_32FC3, Scalar(0,0,0));//XYZ in meters (CV_32FC3)
	float pclX;
	float pclY;
	float pclZ;
	const float factor = 0.001;
    int y=0;
    while(y<ymax){
    	int x=0;
		while(x<xmax){
			pclZ = (float)((depth.at<ushort>(y,x)) * factor);
			pclX = (float)x*factor;
			pclY = (float)y*factor;
			bufferPcl.at<cv::Vec3f>(y,x)[0] = pclX;
			bufferPcl.at<cv::Vec3f>(y,x)[1] = pclY;
			bufferPcl.at<cv::Vec3f>(y,x)[2] = pclZ;
			x++;
		}
    y++;
    }
    cvPcl = bufferPcl.clone();
}
#endif


#ifdef USE_CV
 /**
  * calculate a point cloud in meter via depth image and color camera calibration data (fx,fy, cx, cy)\n
  * input:\n
  * \param	const cv::Mat &depth = opencv depth image
  * Output:\n
  * \param	cv::Mat &cvPcl = opencv point cloud \n
  * opencv format for pcl = cv::Mat kv2::pclKv2(424, 512, CV_32FC3, Scalar(0,0,0));//XYZ in meters (CV_32FC3)\n
  * \return	void
  */
void kv3::calcCvPclCamColor(const cv::Mat &depth, cv::Mat &cvPcl){
    int xmax = depth.size().width;
    int ymax = depth.size().height;
    cv::Mat bufferPcl(ymax, xmax, CV_32FC3, Scalar(0,0,0));//XYZ in meters (CV_32FC3)
	float pclX;
	float pclY;
	float pclZ;
	const float factor = 0.001;
    int y=0;
    while(y<ymax){
    	int x=0;
		while(x<xmax){
			pclZ = (float)((depth.at<ushort>(y,x)) * factor);
			pclX = ((x - cx1) * pclZ / fx1);
			pclY = ((y - cy1) * pclZ / fy1);         
			bufferPcl.at<cv::Vec3f>(y,x)[0] = pclX;
			bufferPcl.at<cv::Vec3f>(y,x)[1] = pclY;
			bufferPcl.at<cv::Vec3f>(y,x)[2] = pclZ;
			x++;
		}
    y++;
    }
    cvPcl = bufferPcl.clone();
}
#endif

#ifdef USE_CV
 /**
  * calculate a point cloud in meter via depth image and infrared camera calibration data (fx,fy, cx, cy)\n
  * input:\n
  * \param	const cv::Mat &depth = opencv depth image
  * Output:\n
  * \param	cv::Mat &cvPcl = opencv point cloud \n
  * opencv format for pcl = cv::Mat kv2::pclKv2(424, 512, CV_32FC3, Scalar(0,0,0));//XYZ in meters (CV_32FC3)\n
  * \return	void
  */
void kv3::calcCvPclCamIr(const cv::Mat &depth, cv::Mat &cvPcl){
    int xmax = depth.size().width;
    int ymax = depth.size().height;
    cv::Mat bufferPcl(ymax, xmax, CV_32FC3, Scalar(0,0,0));//XYZ in meters (CV_32FC3)
	float pclX;
	float pclY;
	float pclZ;
	const float factor = 0.001;
    int y=0;
    while(y<ymax){
    	int x=0;
		while(x<xmax){
			pclZ = (float)((depth.at<ushort>(y,x)) * factor);
			pclX = ((x - cx0) * pclZ / fx0);
			pclY = ((y - cy0) * pclZ / fy0);         
			bufferPcl.at<cv::Vec3f>(y,x)[0] = pclX;
			bufferPcl.at<cv::Vec3f>(y,x)[1] = pclY;
			bufferPcl.at<cv::Vec3f>(y,x)[2] = pclZ;
			x++;
		}
    y++;
    }
    cvPcl = bufferPcl.clone();
}
#endif


#ifdef USE_CV
 /**
  * capture the kinect azure color image.\n
  * 
  * input:\n
  * \param	void  
  * output:\n
  * \param	void / copy internal the data to k4a_image_t kv3:kv3ImColor
  * \return	void
  * \n
  * Error: output to comand line " | Color None   \n                    "
  */
void kv3::capColor(){
        // Probe for a color image
        kv3ImColor = k4a_capture_get_color_image(kv3::capture);
        if (kv3ImColor != NULL)
        {
            // Get the sizes of color image
            int width = k4a_image_get_width_pixels(kv3ImColor);
            int height = k4a_image_get_height_pixels(kv3ImColor);
            //int strides = k4a_image_get_stride_bytes(kv3ImColor);

            // Store the image using opencv Mat
            uint8_t* colorImageData = k4a_image_get_buffer(kv3ImColor);
            if(colorImageData == NULL){
                printf(" ||||||||||||||||||| ERROR - BUFFER: k4a_image_get_buffer");
            }else{
                imgColor = Mat(height, width, CV_8UC4, (void*)colorImageData, Mat::AUTO_STEP);
            }
            k4a_image_release(kv3ImColor);
        }else{
            printf(" | Color None                       ");
        }
        
}
#endif

#ifdef USE_CV
 /**
  * capture the kinect azure on depth map registered color image.\n
  * 
  * input:\n
  * \param	void  
  * output:\n
  * \param	void / registration - create color map for the depth map 
  * \return	void
  * \n
  * Error: output to comand line " | Color None   \n                    "
  */
void kv3::capColorReg(){
    int width = k4a_image_get_width_pixels(kv3ImDepth);
    int height = k4a_image_get_height_pixels(kv3ImDepth);
    k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, width, height, width * 4 * (int)sizeof(uint8_t), &kv3ImColorRegDepth);
    if(K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation, kv3ImDepth, kv3ImColor, kv3ImColorRegDepth))
    {
        cout << "Failed to compute transformed depth image" << endl;
    }

            // Store the image using opencv Mat
            uint8_t* colorImageData = k4a_image_get_buffer(kv3ImColorRegDepth);
            if(colorImageData == NULL){
                printf(" ||||||||||||||||||| ERROR - BUFFER: imgColorRegDepth");
            }else{
                imgColorRegDepth = Mat(height, width, CV_8UC4, (void*)colorImageData, Mat::AUTO_STEP);
            }
            //k4a_image_release(kv3ImColorRegDepth);
}
#endif

#ifdef USE_CV
 /**
  * capture the kinect azure color image MJPEG.\n
  * 
  * input:\n
  * \param	void  
  * output:\n
  * \param	void / copy internal the data to k4a_image_t kv3:kv3ImColor
  * \return	void
  * \n
  * Error: output to comand line " | Color None   \n                    "
  */
void kv3::capColorMJPEG(){
        // Probe for a color image
        kv3ImColor = k4a_capture_get_color_image(kv3::capture);
        if (kv3ImColor != NULL)
        {
            // Get the sizes of color image
            int width = k4a_image_get_width_pixels(kv3ImColor);
            int height = k4a_image_get_height_pixels(kv3ImColor);
            //int strides = k4a_image_get_stride_bytes(kv3ImColor);

            // Store the image using opencv Mat
            uint8_t* colorImageBuffer = k4a_image_get_buffer(kv3ImColor);
            size_t size = k4a_image_get_size( kv3ImColor );

            // decode motion jpeg using opencv
            std::vector<uint8_t> jpeg( colorImageBuffer, colorImageBuffer + size );
            //imgColor = Mat(height, width, CV_8UC4, (void*)colorImageBuffer, Mat::AUTO_STEP);
            imgColor = cv::imdecode( jpeg, cv::IMREAD_ANYCOLOR );

        }else{
            printf(" | Color None                       ");
        }
        
}
#endif

#ifdef USE_CV
 /**
  * capture the kinect azure depth registered color image MJPEG.\n
  * 
  * input:\n
  * \param	void  
  * output:\n
  * \param	void / copy internal the data to k4a_image_t kv3:kv3ImColor
  * \return	void
  * \n
  * Error: output to comand line " | Color None   \n                    "
  */
void kv3::capColorRegMJPEG(){
    int width = k4a_image_get_width_pixels(kv3ImDepth);
    int height = k4a_image_get_height_pixels(kv3ImDepth);
    k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, width, height, width * 4 * (int)sizeof(uint8_t), &kv3ImColorRegDepth);
    if(K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation, kv3ImDepth, kv3ImColor, kv3ImColorRegDepth))
    {
        cout << "Failed to compute transformed depth image" << endl;
    }
            // Store the image using opencv Mat
            uint8_t* colorImageBuffer = k4a_image_get_buffer(kv3ImColorRegDepth);
            size_t size = k4a_image_get_size( kv3ImColorRegDepth );
            if(colorImageBuffer == NULL){
                printf(" ||||||||||||||||||| ERROR - BUFFER: imgColorRegDepth MJPEG");
            }else{
                // decode motion jpeg using opencv
                std::vector<uint8_t> jpeg( colorImageBuffer, colorImageBuffer + size );
                imgColorRegDepth = cv::imdecode( jpeg, cv::IMREAD_ANYCOLOR );
            }
            k4a_image_release(kv3ImColorRegDepth);
}
#endif

#ifdef USE_CV
 /**
  * capture the kinect azure infrared image.
  * 
  * input:\n
  * \param	void  
  * output:\n
  * \param	void / copy internal the data to k4a_image_t kv3:kv3ImIr
  * \return	void
  * \n
  * Error: output to comand line " | IR None"\n
  */
void kv3::capIr(){
    kv3ImIr = k4a_capture_get_ir_image(kv3::capture);
    if (kv3ImIr != NULL)
    {
        // Get the sizes of depth image
        int width = k4a_image_get_width_pixels(kv3ImIr);
        int height = k4a_image_get_height_pixels(kv3ImIr);
        //int strides = k4a_image_get_stride_bytes(kv3ImIr);
        //printf("IR image height, width and strides: %d, %d, %d\n", height, width, strides);

        // Store the image using opencv Mat
        uint16_t* irImageData = (uint16_t*)(void*)k4a_image_get_buffer(kv3ImIr);
        imgIr = Mat(height, width, CV_16U, (void*)irImageData, Mat::AUTO_STEP);

    }else{
        printf(" | IR None\n");
    }
}
#endif

#ifdef USE_CV
 /**
  * capture the kinect azure depth image.\n
  * 
  * input:\n
  * \param	void  
  * output:\n
  * \param	void / copy internal the data to k4a_image_t kv3:kv3ImDepth
  * \return	void
  * \n
  * Error: output to comand line: " | Depth16 None"\n
  *
  */
void kv3::capDepth(){
        kv3ImDepth = k4a_capture_get_depth_image(capture);
        if (kv3ImDepth != NULL)
        {
            // Get the sizes of depth image
            int width = k4a_image_get_width_pixels(kv3ImDepth);
            int height = k4a_image_get_height_pixels(kv3ImDepth);
            //int strides = k4a_image_get_stride_bytes(kv3ImDepth);
            //printf("Depth image height, width and strides: %d, %d, %d\n", height, width, strides);

            // Store the image using opencv Mat
            uint16_t* depthImageData = (uint16_t*)(void*)k4a_image_get_buffer(kv3ImDepth);
            imgDepth = Mat(height, width, CV_16U, (void*)depthImageData, Mat::AUTO_STEP);            
        }else{
            printf(" | Depth16 None\n");
        }
}
#endif


#ifdef USE_CV
void kv3::capPclDepth(){
    int width;
    int height;

    width = k4a_image_get_width_pixels(kv3ImDepth);
    height = k4a_image_get_height_pixels(kv3ImDepth);
    
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        width,
        height,
        width * 3 * (int)sizeof(k4a_float3_t),
        &kv3::kv3ImPcl);

    k4a_transformation_depth_image_to_point_cloud(kv3::transformation, kv3::kv3ImDepth, K4A_CALIBRATION_TYPE_DEPTH, kv3ImPcl);
}
#endif


 /**
  * capture the kinect azure registered depth image.\n
  * 
  * input:\n
  * \param	void  
  * output: HwVerRgb HwVerDepth HwVerDepthSensor HwVerAudio HwVerFirmwareBuild HwVerFirmwareSignature\n
  * \param	string
  * \return	void
  * \n
  * Error: output to comand line: imgDepthReg is empty\n
  */
string kv3::getHardwareVersion(){
    std::string strHwVer = string("");
    try{
        k4a_device_get_version( kv3::device , kv3::k4aHwVer_HwVersion);
        uint32_t major, minor, iteration;
        major = k4aHwVer_HwVersion->rgb.major;
        minor = k4aHwVer_HwVersion->rgb.major;
        iteration = k4aHwVer_HwVersion->rgb.major;
        std::string strHwVerRgb = std::string("rgb:") + std::to_string(major) + std::string(":") + std::to_string(minor) + std::string(":") + std::to_string(iteration);

        major = k4aHwVer_HwVersion->depth.major;
        minor = k4aHwVer_HwVersion->depth.major;
        iteration = k4aHwVer_HwVersion->depth.major;
        std::string strHwVerDepth = std::string(";depth:") + std::to_string(major) + std::string(":") + std::to_string(minor) + std::string(":") + std::to_string(iteration);

        major = k4aHwVer_HwVersion->depth_sensor.major;
        minor = k4aHwVer_HwVersion->depth_sensor.major;
        iteration = k4aHwVer_HwVersion->depth_sensor.major;
        std::string strHwVerDepthSensor = std::string(";depth_sensor:") + std::to_string(major) + std::string(":") + std::to_string(minor) + std::string(":") + std::to_string(iteration);

        major = k4aHwVer_HwVersion->audio.major;
        minor = k4aHwVer_HwVersion->audio.major;
        iteration = k4aHwVer_HwVersion->audio.major;
        std::string strHwVerAudio = std::string(";audio:") + std::to_string(major) + std::string(":") + std::to_string(minor) + std::string(":") + std::to_string(iteration);    

        major = k4aHwVer_HwVersion->firmware_build;
        std::string strHwVerFirmwareBuild =  string("");
        if(k4aHwVer_HwVersion->firmware_build == K4A_FIRMWARE_BUILD_RELEASE){
            strHwVerFirmwareBuild = string(";firmware_build=Production firmware");
        }
        if(k4aHwVer_HwVersion->firmware_build == K4A_FIRMWARE_BUILD_DEBUG){
            strHwVerFirmwareBuild = string(";firmware_build=Pre-production firmware");
        }

        major = k4aHwVer_HwVersion->firmware_signature;
        std::string strHwVerFirmwareSignature =  string("");
        if(k4aHwVer_HwVersion->firmware_signature == K4A_FIRMWARE_SIGNATURE_MSFT){
            strHwVerFirmwareSignature = string(";firmware_signature=Microsoft signed firmware");
        }
        if(k4aHwVer_HwVersion->firmware_signature == K4A_FIRMWARE_SIGNATURE_TEST){
            strHwVerFirmwareSignature = string(";firmware_signature=Test signed firmware");
        }
        if(k4aHwVer_HwVersion->firmware_signature == K4A_FIRMWARE_SIGNATURE_UNSIGNED){
            strHwVerFirmwareSignature = string(";firmware_signature=Unsigned firmware");
        }        
        strHwVer = strHwVerRgb + strHwVerDepth + strHwVerDepthSensor + strHwVerAudio + strHwVerFirmwareBuild + strHwVerFirmwareSignature;

    }catch(string e){
        strHwVer = string("Exeption: ") + e;
    }

    return(strHwVer);
}

 /**
  * capture the kinect azure IMU temperature\n
  * 
  * input:\n
  * \param	void  
  * output: update internal variable temperature value.\n
  * \param	value by reference: float &temperature
  * \return	void
  * 
  */
void kv3::getImuTemperature(float &temperature){
    temperature = imuTemperature;
    //room temp = 18.3  :: value= 10,6 - 10,7

    // °C = (°F - 32) * 5/9
    // °F = °C * 1,8 + 32
}

 /**
  * capture the kinect azure sensor temperature, whitout update the internal variable\n
  * 
  * input:\n
  * \param	void  
  * output: update internal variable temperature value.\n
  * \param	value by reference: float &tempera
  * \return	void
  * 
  */
void kv3::getCapTemperature(float &tempera){
    tempera = (float)k4a_capture_get_temperature_c(capture);
    //room temp = 18.3  :: value= 25,6 - 26,4
   // °C = (°F - 32) * 5/9
   // °F = °C * 1,8 + 32
}

 /**
  * capture the kinect azure sensor temperature\n
  * 
  * input:\n
  * \param	void  
  * output: update internal variable capTemparature value.\n
  * \param	void
  * \return	void
  * 
  */
void kv3::capTemperature(){
    capTemparature = (float)k4a_capture_get_temperature_c(capture);
}

 /**
  * capture the kinect azure IMU sensor\n
  * \n
  * input:\n
  * \param	void  
  * output: update internal variable\n
  * - imuAcc: accelaration\n
  * - imuGravity\n
  * - imuTemperature\n
  * \param	void
  * \return	void
  * 
  * Error: comand line: "Failed to read a imu sample"\n
  */
inline void kv3::updateIMU()
{
    // Capture an IMU sample
    int32_t imu_timeout_ms = 60; 
    switch (k4a_device_get_imu_sample(device, &imu_sample, imu_timeout_ms))
    {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            std::cout <<"Timed out waiting for a imu sample" << endl;
        case K4A_WAIT_RESULT_FAILED:
            std::cout << "Failed to read a imu sample" << endl;
    }
    // Access the accelerometer readings
    imuAcc = imu_sample.acc_sample; // < Accelerometer sample in meters per second squared.
    imuGravity = extract_gravity_from_imu(imuAcc); 
    imuTemperature = (float)imu_sample.temperature; // < Temperature reading of this sample (Celsius). 
    // < Raw -  Gyro sample in radians per second. 
    //cout << "Raw - Gyro x: " << imu_sample.gyro_sample.xyz.x << " y: " << imu_sample.gyro_sample.xyz.y << " z: " << imu_sample.gyro_sample.xyz.z << endl;
    //cout << "Raw - Gyro z: " << imu_sample.gyro_sample.xyz.z << endl;
}

 /**
  * calculate gravity vector based on IMU acceleration vector
  * 
  * input:acceleration vector\n
  * \param	imuAcc  
  * output: calculate gravity vector\n
  *  vector depthGravity x,y,z
  * \param	void
  * \return	void
  * 
  * Error: comand line: "Failed to read a imu sample"\n
  */
inline k4a::Vector kv3::extract_gravity_from_imu(k4a::Vector& imuAcc)
{
    const float* transformation_color_IMU_R = calibration.extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_COLOR].rotation;
    k4a::Vector Rx = { transformation_color_IMU_R[0], transformation_color_IMU_R[1], transformation_color_IMU_R[2] };
    k4a::Vector Ry = { transformation_color_IMU_R[3], transformation_color_IMU_R[4], transformation_color_IMU_R[5] };
    k4a::Vector Rz = { transformation_color_IMU_R[6], transformation_color_IMU_R[7], transformation_color_IMU_R[8] };

    k4a::Vector depthAcc = { Rx.Dot(imuAcc), Ry.Dot(imuAcc) , Rz.Dot(imuAcc) };
    // The acceleration due to gravity, g, is in a direction toward the ground.
    // However an accelerometer at rest in a gravity field reports upward acceleration
    // relative to the local inertial frame (the frame of a freely falling object).
    k4a::Vector depthGravity = depthAcc.Normalized() * -1;

    return depthGravity;
}

 /**
  * get accelartion vector\n
  * \n
  * input:void\n
  * \param	void  
  * output: accelaration vector\n
  * \param	k4a::Vector& vec
  * \return	void
  */
void kv3::getIMU(k4a::Vector& vec){
    vec = imuAcc;//.Normalized();
    //std::cout << "IMU Acc : " << imu_Acc.X << " " << imu_Acc.Y << " " << imu_Acc.Z << std::endl;
    //std::cout << "After : " << vec.X << " " << vec.Y << " " << vec.Z << std::endl;
}

 /**
  * get accelartion float values\n
  * \n
  * input:void\n
  * \param	void  
  * output: accelaration float values\n
  * \param	float &x, float &y, float &z
  * \return	void
  */
void kv3::getIMU(float &x, float &y, float &z){
    x = kv3::imuAcc.X;
    y = kv3::imuAcc.Y;
    z = kv3::imuAcc.Z;
}

 /**
  * get accelartion data in float and degree\n
  * \n
  * input:void\n
  * \param	void  
  * output: accelaration in float degree \n
  * \param	float &x, float &y, float &z
  * \return	void
  */
void kv3::getIMUinDeg(float &x, float &y, float &z){
    float fc = 9;
    x = kv3::imuAcc.X*fc;
    y = kv3::imuAcc.Y*fc;
    z = kv3::imuAcc.Z*fc;
}

 /**
  * get gravity vector\n
  * \n
  * input:void\n
  * \param	void  
  * output: gravity vector\n
  * \param	k4a::Vector& vec
  * \return	void
  */
void kv3::getGravity(k4a::Vector& vec){
    vec = imuGravity;//.Normalized();
    //std::cout << "Before : " << imu_grav.X << " " << imu_grav.Y << " " << imu_grav.Z << std::endl;
    //std::cout << "After : " << vec.X << " " << vec.Y << " " << vec.Z << std::endl;

    //k4a::Vector angle = vec.Angle(k4a::Vector(0,1.0,0));
    //std::cout << "After : " << vec.X << " " << vec.Y << " " << vec.Z << std::endl;
}

 /**
  * get gravity vector\n
  * \n
  * input:void\n
  * \param	void  
  * output: gravity vector\n
  * \param	float &x, float &y, float &z
  * \return	void
  */
void kv3::getGravity(float &x, float &y, float &z){
    x = kv3::imuGravity.X;
    y = kv3::imuGravity.Y;
    z = kv3::imuGravity.Z;
}

void kv3::getXYZrotationValuesInDeg(float &x, float &y, float &z){
    //https://ahrs.readthedocs.io/en/latest/filters/tilt.html
    float ax = kv3::imuAcc.X;
    float ay = kv3::imuAcc.Y;
    float az = kv3::imuAcc.Z;
    float mx = kv3::imuGravity.X;
    float my = kv3::imuGravity.Y;
    float mz = kv3::imuGravity.Z;
    //Roll
    float roll = atan2(ay,az);
    //Pitch
    float pitchIn = sqrt( (ay*ay) + (az*az)  );
    float pitch = atan2(-ax,pitchIn);
    //Tilt
    float tilt = atan2( mz*sin(pitch) - my*cos(pitch), mx*cos(roll) + sin( my*sin(pitch)+mz*cos(pitch) ));  //atan2(-by,bx)

    x = roll *  (float)180.0 / M_PI;
    y = pitch * (float)180.0 / M_PI;
    z = tilt *  (float)180.0 / M_PI;

}

#ifdef USE_CV
void kv3::getIntrinsicIr(cv::Mat &camMatrix, cv::Mat &coeff){
    camMatrix = camMatrixKv3Ir;
    coeff = distCoeffsKv3Ir;
}

void kv3::getIntrinsicColor(cv::Mat &camMatrix, cv::Mat &coeff){
	camMatrix = camMatrixKv3Color;
	coeff = distCoeffsKv3Color;
}

 /**
  * deliver the principal point Color in int values\n
  * \n
  * input:void\n
  * \param	void
  * output: principal point as integer point \n
  * \param	void
  * \return	void
  */
void kv3::getPrincipalPointColor(Point &po){
    po.x = (int)ceil(cx1);
    po.y = (int)ceil(cy1);
}

 /**
  * deliver the principal point Color in float values\n
  * \n
  * input:void\n
  * \param	void
  * output: principal point as float point \n
  * \param	void
  * \return	void
  */
void kv3::getPrincipalPoint2fColor(Point2f &po){
    po.x = (float)cx1;
    po.y = (float)cy1;
}

 /**
  * deliver the principal point IR in int values\n
  * \n
  * input:void\n
  * \param	void
  * output: principal point as integer point \n
  * \param	void
  * \return	void
  */
void kv3::getPrincipalPointIr(Point &po){
    po.x = (int)ceil(cx0);
    po.y = (int)ceil(cy0);
}

 /**
  * deliver the principal point IR in float values\n
  * \n
  * input:void\n
  * \param	void
  * output: principal point as float point \n
  * \param	void
  * \return	void
  */
void kv3::getPrincipalPoint2fIr(Point2f &po){
    po.x = (float)cx0;
    po.y = (float)cy0;
}



 /**
  * draw into image the principal point from color as cross lines in gray color\n
  * \n
  * input:void\n
  * \param	cv::Mat &img
  * output: img with cross lines \n
  * \param	void
  * \return	void
  */
void kv3::drawPrincipalPointColor(const cv::Mat &in, cv::Mat &out){
    out = in.clone();

    Point p1 = Point(ceil(cx1),0);
    Point p2 = Point(ceil(cx1), in.size().height);
    line(out, p1, p2, Scalar(125, 125, 125), 1, LINE_8);
    Point p3 = Point(0,ceil(cy1));
    Point p4 = Point(in.size().width,ceil(cy1));
    line(out, p3, p4, Scalar(125, 125, 125), 1, LINE_8);
}

 /**
  * draw into image the principal point from IR as cross lines in gray color\n
  * \n
  * input:void\n
  * \param	cv::Mat &img
  * output: img with cross lines \n
  * \param	void
  * \return	void
  */
void kv3::drawPrincipalPointIR(const cv::Mat &in, cv::Mat &out){
    out = in.clone();

    Point p1 = Point(ceil(cx0),0);
    Point p2 = Point(ceil(cx0), in.size().height);
    line(out, p1, p2, Scalar(125, 125, 125), 1, LINE_8);
    Point p3 = Point(0,ceil(cy0));
    Point p4 = Point(in.size().width,ceil(cy0));
    line(out, p3, p4, Scalar(125, 125, 125), 1, LINE_8);
}

 /**
  * detect the blur level from an image via Fast Fourier Transformation\n
  * \n
  * input:void\n
  * \param	cv::Mat &img
  * output: image by reference\n
  * \param	void
  * \return	int blur level
  */
int kv3::detectCurrentImageBlurStatusViaFFT(const cv::Mat &img, const double threshold){
    int retval = 0;

    Mat imgGray;
    cvtColor(img,imgGray,cv::COLOR_BGRA2GRAY);

    const int BLOCK = 60;
    int cx = imgGray.cols/2;
    int cy = imgGray.rows/2;

    // Go float
    Mat fImage;
    imgGray.convertTo(fImage, CV_32F);

    // FFT
    Mat fourierTransform;
    dft(fImage, fourierTransform, DFT_SCALE|DFT_COMPLEX_OUTPUT);

    //center low frequencies in the middle
    //by shuffling the quadrants.
    Mat q0(fourierTransform, Rect(0, 0, cx, cy));       // Top-Left - Create a ROI per quadrant
    Mat q1(fourierTransform, Rect(cx, 0, cx, cy));      // Top-Right
    Mat q2(fourierTransform, Rect(0, cy, cx, cy));      // Bottom-Left
    Mat q3(fourierTransform, Rect(cx, cy, cx, cy));     // Bottom-Right

    Mat tmp;                                            // swap quadrants (Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    q1.copyTo(tmp);                                     // swap quadrant (Top-Right with Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);

    // Block the low frequencies
    // #define BLOCK could also be a argument on the command line of course
    fourierTransform(Rect(cx-BLOCK,cy-BLOCK,2*BLOCK,2*BLOCK)).setTo(0);

    //shuffle the quadrants to their original position
    Mat orgFFT;
    fourierTransform.copyTo(orgFFT);
    Mat p0(orgFFT, Rect(0, 0, cx, cy));       // Top-Left - Create a ROI per quadrant
    Mat p1(orgFFT, Rect(cx, 0, cx, cy));      // Top-Right
    Mat p2(orgFFT, Rect(0, cy, cx, cy));      // Bottom-Left
    Mat p3(orgFFT, Rect(cx, cy, cx, cy));     // Bottom-Right

    p0.copyTo(tmp);
    p3.copyTo(p0);
    tmp.copyTo(p3);

    p1.copyTo(tmp);                                     // swap quadrant (Top-Right with Bottom-Left)
    p2.copyTo(p1);
    tmp.copyTo(p2);

    // IFFT
    Mat invFFT;
    Mat logFFT;
    double minVal,maxVal;

    dft(orgFFT, invFFT, DFT_INVERSE|DFT_REAL_OUTPUT);

    invFFT = cv::abs(invFFT);
    cv::minMaxLoc(invFFT,&minVal,&maxVal,NULL,NULL);
    
    //check for impossible values
    if(maxVal<=0.0){
        //cerr << "No information, complete black image!\n";
        return 1;
    }

    cv::log(invFFT,logFFT);
    logFFT *= 20;

    //result = numpy.mean(img_fft)
    cv::Scalar result= cv::mean(logFFT);
    //cout << "Result : "<< result.val[0] << endl;
    //cout << "Blurry --- var:" << variance << endl;

    double variance = result.val[0];
    
    if(threshold >= 0){
        if (variance <= threshold) {
            // Blurry
            //cout << "Blurry --- var:" << variance << endl;
            retval = 1;
        } else {
            // Not blurry
            retval = 0;
        }
    }else{
        if (variance >= threshold) {
            // Blurry
            //cout << "Blurry --- var:" << variance << endl;
            retval = 1;
        } else {
            // Not blurry
            retval = 0;
        }
    }

    // show if you like
    Mat finalImage;
    logFFT.convertTo(finalImage, CV_8U);    // Back to 8-bits

    return(retval);
}


 /**
  * detect the blur level from an image via Laplacian\n
  * \n
  * input:void\n
  * \param	cv::Mat &img
  * output: image by reference\n
  * \param	void
  * \return	int blur level
  */
int kv3::detectCurrentImageBlurStatusViaLaplacian(const cv::Mat &img){
    int retval = 0;
    Mat imgGray;
    cvtColor(img,imgGray,cv::COLOR_BGRA2GRAY);
    Mat imLaplacian;
    Laplacian(imgGray, imLaplacian, CV_64F);
    Scalar mean, stddev; // 0:1st channel, 1:2nd channel and 2:3rd channel
    meanStdDev(imLaplacian, mean, stddev, Mat());
    double variance = stddev.val[0] * stddev.val[0];

    const double threshold = 200;
    if (variance <= threshold) {
        // Blurry
        //cout << "Blurry --- var:" << variance << endl;
        retval = 1;
    } else {
        // Not blurry
        retval = 0;
    }

    return(retval);
}


 /**
  * get color image as opencv image\n
  * \n
  * input:void\n
  * \param	void  
  * output: image by reference\n
  * \param	cv::Mat &img
  * \return	void
  */
void kv3::getCvColorImg(cv::Mat &img){
    imgColor.copyTo(img);
}
#endif

#ifdef USE_CV
 /**
  * get infrared image as opencv image\n
  * \n
  * input:void\n
  * \param	void  
  * output: image by reference\n
  * \param	cv::Mat &img
  * \return	void
  */
void kv3::getCvIrImg(cv::Mat &img){
    imgIr.copyTo(img);
}
#endif

#ifdef USE_CV
 /**
  * get depth image as opencv image\n
  * \n
  * input:void\n
  * \param	void  
  * output: image by reference\n
  * \param	cv::Mat &img
  * \return	void
  */
void kv3::getCvDepthImg(cv::Mat &img){
    imgDepth.copyTo(img);
}
#endif

#ifdef USE_CV
 /**
  * get regidtered depth image as opencv image\n
  * \n
  * input:void\n
  * \param	void  
  * output: image by reference\n
  * \param	cv::Mat &img
  * \return	void
  */
void kv3::getCvRegDepthImg(cv::Mat &img){
    imgColorRegDepth.copyTo(img);
}
#endif


#ifdef USE_CV
 /**
  * calc from depth image the gradients image\n
  * \n
  * input:void\n
  * \param	void  
  * output: image by reference\n
  * \param	cv::Mat &img
  * \return	void
  */
void kv3::calcCvDepthGradientImg(const cv::Mat &img, cv::Mat &imgOut){
        int xmax = img.size().width;
        int ymax = img.size().height;   
        int x=0;
        int y=0; 
        Mat imgOut1(img.size().height,img.size().width,cv::DataType<uchar>::type);
        Mat imgOut2(img.size().height,img.size().width,cv::DataType<uchar>::type);
        Mat imgOut3(img.size().height,img.size().width,cv::DataType<uchar>::type);
        Mat imgOutResult(img.size().height,img.size().width,cv::DataType<uchar>::type);

        double maxGradient1 = 0;
        double maxGradient2 = 0;
        double maxGradient3 = 0;

        unsigned int gLowLimit = 8;
        unsigned int fc = 40;

        x=0;
        y=0;
        while(y<ymax-1){
            x=0;
            while(x<xmax){
                unsigned int gradient1 = (unsigned int)ceil(abs(img.at<ushort>(y,x)-img.at<ushort>(y+1,x)));
                imgOut1.at<uchar>(y,x) = gradient1;
                if(gradient1 < gLowLimit){ gradient1 = gradient1 * fc; }else{gradient1 = 0; }
                if(gradient1 > maxGradient1){maxGradient1 = gradient1;}
                x++;
            }
        y++;
        }
        /////////////////////////////////////////////////////////////
        x=0;
        y=0;
        while(y<ymax-1){
            x=0;
            while(x<xmax-1){
                unsigned int gradient2 = (unsigned int)ceil(abs(img.at<ushort>(y,x)-img.at<ushort>(y,x+1)));
                imgOut2.at<uchar>(y,x) = gradient2;
                if(gradient2 < gLowLimit){ gradient2 = gradient2 * fc; }else{gradient2 = 0; }
                if(gradient2 > maxGradient2){maxGradient2 = gradient2;}
                x++;
            }
        y++;
        }
        /////////////////////////////////////////////////////////////
        x=0;
        y=0;
        while(y<ymax-1){
            x=0;
            while(x<xmax-1){
                unsigned int gradient3 = (unsigned int)ceil(abs(img.at<ushort>(y,x)-img.at<ushort>(y+1,x+1)));
                imgOut3.at<uchar>(y,x) = gradient3;
                if(gradient3 < gLowLimit){ gradient3 = gradient3 * fc; }else{gradient3 = 0; }
                if(gradient3 > maxGradient3){maxGradient3 = gradient3;}
                x++;
            }
        y++;
        }        
        /////////////////////////////////////////////////////////////
        double factor = (maxGradient1+maxGradient2+maxGradient3) / 255;

        x=0;
        y=0;
        while(y<ymax-1){
            x=0;
            while(x<xmax-1){
                imgOutResult.at<uchar>(y,x) = (unsigned int)ceil((imgOut1.at<uchar>(y,x)+imgOut2.at<uchar>(y,x)+imgOut3.at<uchar>(y,x))*factor);                
                x++;
            }
        y++;
        }        
    imgOut = imgOutResult.clone();
}
#endif

#ifdef USE_CV
 /**
  * crop depth and color image - center based crop\n
  * \n
  * input: w = width of the new images and h = height of the new images\n
  * \param	cv::Mat &imColor, cv::Mat &imDepth, const int w, const int h
  * output: resized images\n
  * \param	cv::Mat &imColor, cv::Mat &imDepth
  * \return	void
  */
void kv3::cropKv3Img(cv::Mat &imColor, cv::Mat &imDepth, const int w, const int h){
    if((imgColor.empty() == false)&&(imgColorRegDepth.empty()==false)){
        int maxW = imgColor.size().width;
        int maxH = imgColor.size().height;
        if((w > maxW)||(h > maxH)){
            imgColor.copyTo(imColor);
            imgColorRegDepth.copyTo(imgColorRegDepth);
            return;
        }else{
            int uX = (int)(round(maxW*0.5)) - (int)(round(w*0.5));
            int uY = (int)(round(maxH*0.5)) - (int)(round(h*0.5));
            cv::Rect roi = cv::Rect(uX,uY,w,h);
            cv::Mat resizeColor = imgColor(roi);
            imColor = resizeColor.clone();
            cv::Mat resizeDepthReg = imgColorRegDepth(roi);
            imDepth = resizeDepthReg.clone();
        }
    }//if img not empty   
}
#endif

#ifdef USE_CV
 /**
  * resize depth and color image to the same size\n
  * \n
  * input: w = width of the new images and h = height of the new images\n
  * in case h OR w = 0, then calculate the function the missing value to get a image with correct ratio.\n
  * \param	const int inW, const int inH
  * output: resized images\n
  * \param	cv::Mat &imSrcRgb, cv::Mat &imDepth,
  * \return	void
  */
void kv3::resizeKv3Img(cv::Mat &imSrcRgb, cv::Mat &imDepth, const int inW, const int inH){
    if((imSrcRgb.empty() == false)&&(imgColorRegDepth.empty()==false)){
        int w = inW;
        int h = inH;

        if(w <= 0){
            //calculate w
            int mW = imSrcRgb.size().width;
            int mH = imSrcRgb.size().height;
            double ratioWH = double((double)mW/(double)mH);
            w = (int)round((double)h * ratioWH);
        }

        if(h <= 0){
            //calculate h
            int mW = imSrcRgb.size().width;
            int mH = imSrcRgb.size().height;
            double ratioWH = double((double)mH/(double)mW);
            h = (int)round((double)w * ratioWH);           
        }

        cv::Mat resizeColor;
        cv::Size destSize = cv::Size(w,h);
        cv::resize(imSrcRgb, resizeColor, destSize, cv::INTER_CUBIC);
        imSrcRgb = resizeColor.clone();
        cv::Mat resizeDepthReg;
        cv::resize(imgColorRegDepth, resizeDepthReg, destSize, cv::INTER_CUBIC);
        imDepth = resizeDepthReg.clone();
    }//if img not empty   
}
#endif

#ifdef USE_CV
 /**
  * calculate the point cloud based on the depth image and calibration data (fx,fy,cx,cy)\n
  * \n
  * input:depth image; width and the height of the depth image; calibration data source: true = color cam calibration; false = infrared calibartion data\n
  * \param	const cv::Mat &inDepth,  const int tarW, const int tarH, const bool bColor
  * output: point cloud\n
  * \param	cv::Mat &cvPcl,
  * \return	void
  */
void kv3::calcCvPclCamResize(const cv::Mat &inDepth, cv::Mat &cvPcl, const int tarW, const int tarH, const bool bColor){
    //calculate calibration data new (cx, cy, fx, fy)
    if(inDepth.empty() == false){
        int w = tarW;
        int h = tarH;
        double ratioH = 0;
        double ratioW = 0;

        if(w <= 0){ w = inDepth.size().width; }
        if(h <= 0){ h = inDepth.size().height; }

        if(w > inDepth.size().width){ w = inDepth.size().width; }
        if(h > inDepth.size().height){ h = inDepth.size().height; }

        if(bColor == true){
            ratioW = double((double)w/(double)(calibration.color_camera_calibration.resolution_width));
            ratioH = double((double)h/(double)(calibration.color_camera_calibration.resolution_height));            
        }else{   
            ratioW = double((double)w/(double)(calibration.depth_camera_calibration.resolution_width));
            ratioH = double((double)h/(double)(calibration.depth_camera_calibration.resolution_height));                                             
        }

        float cxr;
        float cyr;
        float fxr;
        float fyr;
        if(bColor == true){
            cxr = kv3::cx1 * ratioW;
            cyr = kv3::cy1 * ratioH;
            fxr = kv3::fx1 * ratioW;
            fyr = kv3::fy1 * ratioH;
        }else{
            cxr = kv3::cx0 * ratioW;
            cyr = kv3::cy0 * ratioH;
            fxr = kv3::fx0 * ratioW;
            fyr = kv3::fy0 * ratioH;
        }
        int xmax = inDepth.size().width;
        int ymax = inDepth.size().height;
        cv::Mat bufferPcl(ymax, xmax, CV_32FC3, Scalar(0,0,0));//XYZ in meters (CV_32FC3)
        float pclX;
        float pclY;
        float pclZ;
        const float factor = 0.001;
        int y=0;
        while(y<ymax){
            int x=0;
            while(x<xmax){
                pclZ = (float)((inDepth.at<ushort>(y,x)) * factor);
                pclX = ((x - cxr) * pclZ / fxr);
                pclY = ((y - cyr) * pclZ / fyr);         
                bufferPcl.at<cv::Vec3f>(y,x)[0] = pclX;
                bufferPcl.at<cv::Vec3f>(y,x)[1] = pclY;
                bufferPcl.at<cv::Vec3f>(y,x)[2] = pclZ;
                x++;
            }
        y++;
        }
        cvPcl = bufferPcl.clone();
    }//if img not empty   
}
#endif

#ifdef USE_CV
 /**
  * resize color image\n
  * \n
  * input: width and height from the new size\n
  * \param	const int inW, const int inH
  * output: imColor\n
  * \param	cv::Mat &imColor,
  * \return	void
  */
void kv3::resizeKv3ImgColor(cv::Mat &imColor, const int inW, const int inH){
    if(imgColor.empty() == false){
        int w = inW;
        int h = inH;

        if(w <= 0){
            //calculate w
            int mW = imgColor.size().width;
            int mH = imgColor.size().height;
            double ratioWH = double((double)mW/(double)mH);
            w = (int)round((double)h * ratioWH);
        }

        if(h <= 0){
            //calculate h
            int mW = imgColor.size().width;
            int mH = imgColor.size().height;
            double ratioWH = double((double)mH/(double)mW);
            h = (int)round((double)w * ratioWH);           
        }

        cv::Mat resizeColor;
        cv::Size destSize = cv::Size(w,h);
        cv::resize(imgColor, resizeColor, destSize, cv::INTER_CUBIC);
        imColor = resizeColor.clone();
    }//if img not empty   
}
#endif

#ifdef USE_CV
 /**
  * resize image\n
  * \n
  * input: width and height from the new size\n
  * \param	const int inW, const int inH
  * output: imColor\n
  * \param	cv::Mat &img,
  * \return	void
  */
void kv3::resizeImg(cv::Mat &img, const int inW, const int inH){
    if(img.empty() == false){
        int w = inW;
        int h = inH;

        if(w <= 0){
            //calculate w
            int mW = img.size().width;
            int mH = img.size().height;
            double ratioWH = double((double)mW/(double)mH);
            w = (int)round((double)h * ratioWH);
        }

        if(h <= 0){
            //calculate h
            int mW = img.size().width;
            int mH = img.size().height;
            double ratioWH = double((double)mH/(double)mW);
            h = (int)round((double)w * ratioWH);           
        }

        cv::Mat resizeColor;
        cv::Size destSize = cv::Size(w,h);
        cv::resize(img, resizeColor, destSize, cv::INTER_CUBIC);
        img = resizeColor.clone();
    }//if img not empty       
}
#endif


#ifdef USE_CV
 /**
  * crop infrared image - ceneter based\n
  * \n
  * input: width and height from the new size\n
  * \param	const int inW, const int inH
  * output: croped image\n
  * \param	cv::Mat &img,
  * \return	void
  */
void kv3::cropKv3IrImg(cv::Mat &imIr, const int w, const int h){
    if(imgIr.empty() == false){
        int maxW = imgIr.size().width;
        int maxH = imgIr.size().height;
        if((w > maxW)||(h > maxH)){
            imgIr.copyTo(imIr);
            return;
        }else{
            int uX = (int)(round(maxW*0.5)) - (int)(round(w*0.5));
            int uY = (int)(round(maxH*0.5)) - (int)(round(h*0.5));
            cv::Rect roi = cv::Rect(uX,uY,w, h);
            cv::Mat resizeColor = imgIr(roi);
            imIr = resizeColor.clone();
        }
    }//if img not empty   
}	
#endif


#ifdef USE_CV
 /**
  * resize image\n
  * \n
  * input: width and height from the new size\n
  * \param	const int inW, const int inH
  * output: resized image\n
  * \param	cv::Mat &imIr,
  * \return	void
  */
void kv3::resizeKv3IrImg(cv::Mat &imIr, const int inW, const int inH){
    if(imgIr.empty() == false){
        int w = inW;
        int h = inH;

        if(w <= 0){
            //calculate w
            int mW = imgIr.size().width;
            int mH = imgIr.size().height;
            double ratioWH = double((double)mW/(double)mH);
            w = (int)round((double)h * ratioWH);
        }

        if(h <= 0){
            //calculate h
            int mW = imgIr.size().width;
            int mH = imgIr.size().height;
            double ratioWH = double((double)mH/(double)mW);
            h = (int)round((double)w * ratioWH);           
        }

        cv::Mat resizeIr;
        cv::Size destSize = cv::Size(w,h);
        cv::resize(imgColor, resizeIr, destSize, cv::INTER_CUBIC);
        imIr = resizeIr.clone();
    }//if img not empty   
}
#endif

#ifdef USE_CV
 /**
  * converte 16 bit depth image to a visible image\n
  * \n
  * input: depth values\n
  * \param	const cv::Mat &in
  * output: visible image\n
  * \param	cv::Mat &out
  * \return	void
  */
void kv3::convToVisibleImg(const cv::Mat &in, cv::Mat &out){
    out = in.clone();
    int maxValue = 0;
    int mW = in.size().width;
    int mH = in.size().height;

    int y=0;
    int x=0;
    y=0;
    while(y<mH){
    	x=0;
		while(x<mW){	
            int val = (int)(in.at<ushort>(y,x));
            if(val > maxValue){ maxValue = val; }
            //dest.at<ushort>(y,x) = (int)(src.at<ushort>(y,x))*(factor);
			x++;
		}
		y++;
    }

    float factor = (65536/maxValue);

    y=0;
    while(y<mH){
    	x=0;
		while(x<mW){
            out.at<ushort>(y,x) = (int)round((in.at<ushort>(y,x))*(factor));
			x++;
		}
		y++;
    }

}
#endif

#ifdef USE_CV
 /**
  * converte 16 bit infrared image to a visible image \n
  * \n
  * input: infrared values\n
  * \param	const cv::Mat &src
  * output: visible image\n
  * \param	cv::Mat &dest
  * \return	void
  */
void kv3::convIrToVisibleImg(const cv::Mat &src, cv::Mat &dest){
    dest = src.clone();
    //  12bit=2^12=4096 // 14bit=2^14=16384 // 16bit=2^16=65536 / 131072
    const float resolution = 16384-1;
    float factor;
    float divisor;

    if(config.depth_mode == K4A_DEPTH_MODE_NFOV_2X2BINNED){
        divisor = (float)5460.0;
    }
    if(config.depth_mode == K4A_DEPTH_MODE_NFOV_UNBINNED){
        divisor = (float)3860.0;
    }
    if(config.depth_mode == K4A_DEPTH_MODE_WFOV_2X2BINNED){
        divisor = (float)2880.0;
    }
    if(config.depth_mode == K4A_DEPTH_MODE_WFOV_UNBINNED){
        divisor = (float)2210.0;
    }         
    factor = (float)(65536/divisor);

    int mW = src.size().width;
    int mH = src.size().height;

    int y=0;
    y=0;
    while(y<mH){
    	int x=0;
		while(x<mW){	
            dest.at<ushort>(y,x) = (int)(src.at<ushort>(y,x))*(factor);
			x++;
		}
		y++;
    }
}
#endif


#ifdef USE_CV
 /**
  * converte 16 bit image to a visible 8bit gray image\n
  * \n
  * input: infrared values\n
  * \param	const cv::Mat &src
  * output: visible image\n
  * \param	cv::Mat &dest
  * \return	void
  */
void kv3::convIrToVisible8BitGrayImg(const cv::Mat &src, cv::Mat &dest){
    //dest = src.clone();
    
    //  12bit=2^12=4096 // 14bit=2^14=16384 // 16bit=2^16=65536 / 131072
    const float resolution = 16384-1;
    float factor;
    float divisor;

    if(config.depth_mode == K4A_DEPTH_MODE_NFOV_2X2BINNED){
        divisor = (float)5460.0;
    }
    if(config.depth_mode == K4A_DEPTH_MODE_NFOV_UNBINNED){
        divisor = (float)3860.0;
    }
    if(config.depth_mode == K4A_DEPTH_MODE_WFOV_2X2BINNED){
        divisor = (float)2880.0;
    }
    if(config.depth_mode == K4A_DEPTH_MODE_WFOV_UNBINNED){
        divisor = (float)2210.0;
    }         
    factor = (float)(float)65536/(divisor);
    factor = (factor / (float)256.0);

    int mW = src.size().width;
    int mH = src.size().height;

    dest = Mat(mH, mW, CV_8UC1);

    int y=0;
    y=0;
    while(y<mH){
    	int x=0;
		while(x<mW){	
            dest.at<uchar>(y,x) = (uint8_t)(src.at<ushort>(y,x)*factor);
			x++;
		}
		y++;
    }
    equalizeHist( dest, dest ); //8bit channel
}
#endif

#ifdef USE_CV

 /**
  * calc angle between a two 3D vector\n
  * \n
  * input: fix Point 
  * \param	Point3d
  * output: angle x in degree, angle y in degree
  * \param	float, float
  * \return	void
  */
void kv3::calcAngleBetween3DvectorPoints(const Point3f &p1, const Point3f &p2, double &outAngle){
    double skalarProduct = (p1.x*p2.x)+(p1.y*p2.y)+(p1.z*p2.z);
    double P1length = sqrt( (p1.x*p1.x)+(p1.y*p1.y)+(p1.z*p1.z) );
    double P2length = sqrt( (p2.x*p2.x)+(p2.y*p2.y)+(p2.z*p2.z) );
    outAngle = acos( skalarProduct / (P1length*P2length) );
}


 /**
  * calc angle between a 3D point and the Principal point ray\n
  * \n
  * input: fix Point 
  * \param	Point3d
  * output: angle x in degree, angle y in degree
  * \param	float, float
  * \return	void
  */
void kv3::calcDeltaAngleFromPrincipalPointTofixPoint(const Point3d &poFix, float &ax, float &ay){
    Point3f p1; Point3f p2; Point3f p3;
    double angle1; double angle2;

    p1.x = (float)0; 
    p1.y = (float)0;
    p1.z = (float)1000.0;

    p2.x = (float)poFix.x;
    p2.y = (float)0;
    p2.z = (float)poFix.z;

    p3.x = (float)0;
    p3.y = (float)poFix.y;
    p3.z = (float)poFix.z;       

    calcAngleBetween3DvectorPoints(p1, p2, angle1);
    ax = (float)angle1 * (float)180.0 / M_PI;
    calcAngleBetween3DvectorPoints(p1, p3, angle2);
    ay = (float)angle2 * (float)180.0 / M_PI;

    if(poFix.x < 0){ax = ax * -1;}
    if(poFix.y < 0){ay = ay * -1;}
}


 /**
  * save all kv3 CV images to path\n
  * \n
  * input: path to the folder to save all images
  * \param	string path, int the counter
  * output: void
  * \return	void
  */
void kv3::saveKv3Images(string path, long int counter, kv3 &kv3AppObj, int widthOfPcl, Mat &resizeImgColor, Mat &resizeImgColorDepth, Mat &regDepth, Mat &colorImg){
    string strNum = to_string(counter);    
    //string strFilePath0 = string("/var/www/ramdev/kv3_CV2_PCL_color_") + strNum + string(".ply");
    string strFilePath0 = path + strNum + string("_kv3_CV2_PCL_color")  + string(".ply");
    string strFilePath1 = path + strNum + string("_kv3_CV2_PCL_color_big") + string(".ply");
    string strFilePath2 = path + strNum + string("_kv3_MS_gen_PCL") + string(".ply");
    string strFilePath3 = path + strNum + string("_kv3_MS_gen_PCL_Color") + string(".ply");
    string strFilePath4 = path + strNum + string("_kv3_MS_PCL_color") + string(".ply");    

	kv3AppObj.resizeKv3Img(resizeImgColor, resizeImgColorDepth,  widthOfPcl, 0);
    //Color ok - smaller because resized
    Mat kv3PclColor;
    kv3AppObj.calcCvPclCamResize(resizeImgColorDepth, kv3PclColor, widthOfPcl, 0, true);
    cv::viz::writeCloud(strFilePath0, kv3PclColor, resizeImgColor);	

    //ok - but big, but nice mapping
    Mat kv3PclColorBig;
    kv3AppObj.calcCvPclCamColor(regDepth, kv3PclColorBig);
    cv::viz::writeCloud(strFilePath1, kv3PclColorBig, colorImg);		
    /*	
    kv3AppObj.writePCL(strFilePath2.c_str());	//Ok PCL , no color but gen depth fix
    kv3AppObj.writeColorPcl1(strFilePath3.c_str()); // ok, but color doesn't map nice, used MS gen fixed PCL data and map the color, only
    kv3AppObj.writeColorPcl2(strFilePath4.c_str()); // ok, but color doesn't map nice   
    //kv3AppObj.writeColorPcl3("/var/www/ramdev/testColor3.ply", point_count);// nok 
    */
}
#endif

#ifdef USE_CV
 /**
  * save kv3 CV color image to path\n
  * \n
  * input: path to the folder to save the image
  * \param	string path, long int the counter
  * output: void
  * \return	void
  */
void kv3::saveColorImg(string path, long int counter, Mat &colorImg){
    string filename = string("_kv3_CV2_RGB");
    string strNum = to_string(counter);  
    string strTargetPath = path + strNum + filename + ".png";
    imwrite(strTargetPath, colorImg);
}
#endif

#ifdef USE_CV
 /**
  * save kv3 CV RGBD image to path\n
  * \n
  * input: path to the folder to save the image
  * \param	string path, long int the counter
  * output: void
  * \return	void
  */
void kv3::saveRGBDImg(string path, long int counter, Mat &RGB, Mat&D){
    string filenameRGB = string("_kv3_CV2_RGB_refToD");
    string filenameD = string("_kv3_CV2_D_refToRGB");
    string strNum = to_string(counter);  
    string strRGBTargetPath = path + strNum + filenameRGB + ".png";
    string strDTargetPath = path + strNum + filenameD + ".png";
    if(!RGB.empty()){
        imwrite(strRGBTargetPath, RGB);  
    }else{
        cout << "RGB is empty." << endl;
    }
    if(!D.empty()){
        imwrite(strDTargetPath, D);   
    }else{
        cout << "D is empty." << endl;
    }
}
#endif

 /**
  * save kv3 serial nummber, temperature and IMU data to path\n
  * \n
  * input: path to the folder to save all data
  * \param	string path, long int the counter
  * output: csv file:  kv3-Temperature;kv3-IMU-Temperature;AccelX;AccelY;AccelZ;GravityX;GravityY,GravityZ
  * \return	void
  */
void kv3::savekv3data(string path, long int counter){
	FILE *fp_kv3data;
    string filename = string("_kv3Data");
    string strNum = to_string(counter);  
    string strTargetPath = path + strNum + filename + ".csv";
    string strkv3SerialNr = "serial Nr.: " + getSerialNumber() + "\n";
    //string strkv3HwVer = "Hardware Version: " + getHardwareVersion() + "\n";
        
    float temperatureKv3;
	getCapTemperature(temperatureKv3);
    float temperatureKv3Imu;
    getImuTemperature(temperatureKv3Imu);
    float accX; float accY; float accZ;
    getIMU(accX, accY, accZ);
    float graX; float graY; float graZ;
    getGravity(graX, graY, graZ);

	fp_kv3data = fopen(strTargetPath.c_str(),"w");
	if(fp_kv3data == NULL) {
        string strErrMessage = "err: " + strTargetPath + " - not there\n";
	    printf(strErrMessage.c_str());
	}else{
		//write
		//fprintf(fp_kv3data, strkv3SerialNr.c_str());
		//fprintf(fp_kv3data, strkv3HwVer.c_str());
        fprintf(fp_kv3data, "%f;", temperatureKv3);
        fprintf(fp_kv3data, "%f;", temperatureKv3Imu);
        fprintf(fp_kv3data, "%f;%f;%f;", accX, accY, accZ);
        fprintf(fp_kv3data, "%f;%f;%f\n", graX, graY, graZ);
	}
	fclose(fp_kv3data);  
}

 /**
  * save kv3 serial nummber, temperature and IMU data to path\n
  * \n
  * input: path to the folder to save all images
  * \param	string path, long int the counter
  * output: csv file:  kv3-Temperature;kv3-IMU-Temperature;AccelX;AccelY;AccelZ;GravityX;GravityY,GravityZ
  * \return	void
  */
void kv3::savekv3gravitydata(string path, long int counter){
	FILE *fp_kv3data;
    string filename = string("_kv3DataGravity");
    string strNum = to_string(counter);
    string strTargetPath = path + strNum + filename + ".csv";
    float graX; float graY; float graZ;
    getGravity(graX, graY, graZ);

	fp_kv3data = fopen(strTargetPath.c_str(),"w");
	if(fp_kv3data == NULL) {
        string strErrMessage = "err: " + strTargetPath + " - not there\n";
	   printf(strErrMessage.c_str());
	}else{
		//write
        fprintf(fp_kv3data, "%f;%f;%f\n", graX, graY, graZ);
	}
	fclose(fp_kv3data);  
}

#ifdef USE_CV
 /**
  * save kv3 dataset (Gravity data, RGB image, Pointcloud )\n
  * \n
  * input: path to the folder to save all images
  * \param	string path, long int the counter
  * output: csv file:  kv3-Temperature;kv3-IMU-Temperature;AccelX;AccelY;AccelZ;GravityX;GravityY,GravityZ
  * \return	void
  */
void kv3::savekv3Dataset(string path, long int counter, Mat &RGB, Mat&D, kv3 kv3AppObj){
    savekv3gravitydata(path, counter);    
    saveColorImg(path, counter, RGB);

    string strNum = to_string(counter);
    string strFilePath1 = path + strNum + string("_kv3_CV2_PCL_Rgb") + string(".ply");
    string strFilePath2 = path + strNum + string("_kv3_CV2_PCL") + string(".ply");
    
    Mat grabPcl;
    kv3AppObj.calcCvPclCamColor(D, grabPcl);
    cv::viz::writeCloud(strFilePath1, grabPcl, RGB);
}
#endif

 /**
  * save kv3 Pointcloud direct as ply file\n
  * \n
  * input: path to the folder to save all images
  * \param	string path, long int the counter
  * output: ply file
  * \return	void
  */
void kv3::savekv3PclDirect(string path){
    string filename = string("kv3_savekv3PclDirect");
    string filePath = path + filename + ".ply";

    std::vector<color_point_t> points;

    int width = k4a_image_get_width_pixels(kv3::kv3ImPcl);
    int height = k4a_image_get_height_pixels(kv3::kv3ImPcl);
    int16_t *point_cloud_image_data = (int16_t *)(void *)k4a_image_get_buffer(kv3::kv3ImPcl);
    uint8_t *color_image_data = k4a_image_get_buffer(kv3::kv3ImColor); 

    for (int i = 0; i < width * height; i++)
    {
        color_point_t point;
        point.xyz[0] = point_cloud_image_data[3 * i + 0];
        point.xyz[1] = point_cloud_image_data[3 * i + 1];
        point.xyz[2] = point_cloud_image_data[3 * i + 2];
        if (point.xyz[2] == 0)
        {
            continue;
        }

        point.rgb[0] = color_image_data[4 * i + 0];
        point.rgb[1] = color_image_data[4 * i + 1];
        point.rgb[2] = color_image_data[4 * i + 2];
        uint8_t alpha = color_image_data[4 * i + 3];

        if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0)
        {
            continue;
        }

        points.push_back(point);
    }

    std::cout << "Size of the pointcloud: " << points.size() << endl;

#define PLY_START_HEADER "ply"
#define PLY_END_HEADER "end_header"
#define PLY_ASCII "format ascii 1.0"
#define PLY_ELEMENT_VERTEX "element vertex"

    // save to the ply file
    std::ofstream ofs(filePath.c_str()); // text mode first
    ofs << PLY_START_HEADER << std::endl;
    ofs << PLY_ASCII << std::endl;
    ofs << PLY_ELEMENT_VERTEX << " " << points.size() << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "property uchar red" << std::endl;
    ofs << "property uchar green" << std::endl;
    ofs << "property uchar blue" << std::endl;
    ofs << "element face 0" << std::endl;
    ofs << "property list uchar int vertex_indices" << std::endl;    
    ofs << PLY_END_HEADER << std::endl;
    ofs.close();

    std::stringstream ss;
    for (size_t i = 0; i < points.size(); ++i)
    {
        // image data is BGR
        ss << (float)points[i].xyz[0] << " " << (float)points[i].xyz[1] << " " << (float)points[i].xyz[2];
        ss << " " << (float)points[i].rgb[2] << " " << (float)points[i].rgb[1] << " " << (float)points[i].rgb[0];
        ss << std::endl;
    }
    std::ofstream ofs_text(filePath.c_str(), std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

 /**
  * save kv3 Pointcloud direct as pcd file\n
  * \n
  * input: path to the folder to save all images
  * \param	string path, long int the counter
  * output: ply file
  * \return	void
  */
void kv3::savekv3PclDirectAsPcdfile(string path){
    string filename = string("kv3_savekv3PclDirect");
    string filePath = path + filename + ".pcd";

    std::vector<color_point_t> points;

    int width = k4a_image_get_width_pixels(kv3::kv3ImPcl);
    int height = k4a_image_get_height_pixels(kv3::kv3ImPcl);
    int16_t *point_cloud_image_data = (int16_t *)(void *)k4a_image_get_buffer(kv3::kv3ImPcl);
    uint8_t *color_image_data = k4a_image_get_buffer(kv3::kv3ImColor); 

    for (int i = 0; i < width * height; i++)
    {
        color_point_t point;
        point.xyz[0] = point_cloud_image_data[3 * i + 0];
        point.xyz[1] = point_cloud_image_data[3 * i + 1];
        point.xyz[2] = point_cloud_image_data[3 * i + 2];
        if (point.xyz[2] == 0)
        {
            continue;
        }

        point.rgb[0] = color_image_data[4 * i + 0];
        point.rgb[1] = color_image_data[4 * i + 1];
        point.rgb[2] = color_image_data[4 * i + 2];
        uint8_t alpha = color_image_data[4 * i + 3];

        if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0)
        {
            continue;
        }

        points.push_back(point);
    }

    std::cout << "Size of the pointcloud: " << points.size() << endl;


/*
# .PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z rgb
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 213
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 213
DATA ascii

//////////// aus echter datei geholt

# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 4857
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 4857
DATA ascii
*/    
    // save to the pcd file
    std::ofstream ofs(filePath.c_str()); // text mode first
    ofs << "# .PCD v.7 - Point Cloud Data file format" << std::endl;
    ofs << "VERSION .7" << std::endl;
    ofs << "FIELDS x y z r g b" << std::endl;
    ofs << "SIZE 4 4 4 1 1 1" << " " << points.size() << std::endl;
    ofs << "TYPE F F F U U U" << std::endl;
    ofs << "COUNT 1 1 1 1 1 1" << std::endl;
    ofs << "WIDTH " << to_string(width) << std::endl;
    ofs << "HEIGHT " << to_string(height)  << std::endl;
    ofs << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    ofs << "POINTS " << to_string(points.size()) << std::endl;
    ofs << "DATA ascii" << std::endl;
    ofs.close();

    std::stringstream ss;
    for (size_t i = 0; i < points.size(); ++i)
    {
        // image data is BGR
        ss << (float)points[i].xyz[0] << " " << (float)points[i].xyz[1] << " " << (float)points[i].xyz[2];
        ss << " " << (float)points[i].rgb[2] << " " << (float)points[i].rgb[1] << " " << (float)points[i].rgb[0];
        ss << std::endl;
    }
    std::ofstream ofs_text(filePath.c_str(), std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}



 /**
  * get current setup id\n
  * \n
  * input: void
  * \param	void
  * output: void
  * \return	setup id
  */
int kv3::getCurrentSetupId(){
    return(currentSetupId);
}

 /**
  * set current setup by id\n
  * \n
  * input: setup id
  * \param	int
  * output: void
  * \return	void
  */
void kv3::setCurrentSetup(int id){
    switch(id){
        case 1:
        kv3::setup01();
        case 2:
        kv3::setup02();
        case 3:
        kv3::setup03();
        case 4:
        kv3::setup04();
        case 5:
        kv3::setup05();    
        case 6:
        kv3::setup06();        
        case 7:
        kv3::setup07();                                        
        default:
        kv3::setup01();
    }
}


 /**
  * set current FPS mode\n
  * \n
  * input: FPS id
  * \param	int
  * output: void
  * \return	setup id
  */
void kv3::setFpsMode(int mode){
    fpsMode = mode;
    if(fpsMode < 0){fpsMode = 0;}
    if(fpsMode > 2){fpsMode = 2;}
}


 /**
  * set current FPS mode\n
  * \n
  * input: FPS id
  * \param	int
  * output: void
  * \return	fps mode 0=5 FPS; 1=15 FPS; 2=30 FPS
  */
int kv3::getFpsMode(){
    return(fpsMode);
}

#ifdef USE_CV
/**
  * read the point cloud 3D point from kv3 sensor at 2D point position \n
  * 
  * input:\n
  * \param	Point 2D 
  * output:\n
  * \param	Point 3D (internal grab from current kv3 PCL)
  * \return	void
  */
void kv3::getPcl3DPointFrom2DPoint(const Point &inPo2D, Point3d &outPclPoint){
    
    int width = k4a_image_get_width_pixels(kv3ImDepth);
    int height = k4a_image_get_height_pixels(kv3ImDepth);    
    uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(kv3ImDepth);

    k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(kv3_xy_table);
    int widthXYtable = k4a_image_get_width_pixels(kv3_xy_table);
    int heightXYtable = k4a_image_get_height_pixels(kv3_xy_table);

    int i = inPo2D.y*width + inPo2D.x;
    outPclPoint.x = xy_table_data[i].xy.x * depth_data[i];
    outPclPoint.y = xy_table_data[i].xy.y * depth_data[i];
    outPclPoint.z = depth_data[i];
}

/**
  * read 2D point from color image and transform the point to "2D point inside deptg image\n
  * 
  * input:\n
  * \param	Point 2D 
  * output:\n
  * \param	Point 2D
  * \return	bool pointDetected
  */
bool kv3::transformColor2DpointToDepth2Dpoint(const Point &inPo2D, Point &outPo2D){
    bool retvalue = false;
    Mat imgCvPoint = Mat::zeros( imgColor.size(), CV_8UC4); 
    Mat gray;
    k4a_image_t imgSinglePointk4aDummyRGBA = NULL;

    int i = 1;
    int imax = 8;
    while(i<imax){
                
        k4a_image_t imgk4aDummy = NULL;        
        //detect a point with depth
        int poSize = i;
        line(imgCvPoint, inPo2D, inPo2D, Scalar(255, 255, 255), poSize, LINE_8); //set the point inside the image
        convertOpenCvImageToK4aImage(imgCvPoint, 0, imgSinglePointk4aDummyRGBA);
        //transform from 2D RGBA to 2D reg depth map
        int width = k4a_image_get_width_pixels(kv3ImDepth);
        int height = k4a_image_get_height_pixels(kv3ImDepth);
        k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, width, height, width * 4 * (int)sizeof(uint8_t), &imgk4aDummy);
        if(K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation, kv3ImDepth, imgSinglePointk4aDummyRGBA, imgk4aDummy))
        {
            cout << "Failed to compute transformed imgSinglePointk4aDummyRGBA to reg depth image" << endl;
        }
            // Store the image using opencv Mat
            uint8_t* colorImageData = k4a_image_get_buffer(imgk4aDummy);
            if(colorImageData == NULL){
                printf(" ||||||||||||||||||| ERROR - BUFFER: imgk4aDummy");
            }else{
                convertK4aImageToOpenCvImage(imgk4aDummy, 2, gray);
                int x = 0; int xmax = gray.size().width;
                int y = 0; int ymax = gray.size().height;
                while(y<ymax){
                    x=0;
                    while(x<xmax){
                        int value = gray.at<uchar>(y,x);
                        if(value > 0){
                            outPo2D.x = x;
                            outPo2D.y = y;
                            retvalue = true;
                            i = imax;
                            x = xmax;
                            y = ymax;
                        } 
                    x++;
                    }
                y++;
                }
            }

    k4a_image_release(imgSinglePointk4aDummyRGBA);
    k4a_image_release(imgk4aDummy);
    i++;
    }

    return(retvalue);
}
#endif


#ifdef USE_CV
/**
  * convert: k4a_image to openCV CV_8UC3 \n
  * 
  * input:\n
  * \param	in = k4a_image image; int  format: target format 0 = RGBA, 1 = RGB, 2 = Gray
  * output:\n
  * \param	out = cv::Mat image
  * \return	void
  */
void kv3::convertK4aImageToOpenCvImage(const k4a_image_t &in, const int format, cv::Mat &out){
    cv::Mat img = cv::Mat(k4a_image_get_height_pixels(in), k4a_image_get_width_pixels(in), CV_8UC4, k4a_image_get_buffer(in));
    //cv::Mat img = Mat(k4a_image_get_height_pixels(in), k4a_image_get_width_pixels(in), CV_8UC4, (void*)in, Mat::AUTO_STEP);
    if(format == 0){
        out = img.clone();
    }
    if(format == 1){
        cvtColor(img, out, cv::COLOR_RGBA2RGB);
    }
    if(format == 2){
        cvtColor(img, out, cv::COLOR_RGBA2GRAY);
    }
}//end_fct
#endif


#ifdef USE_CV
/**
  * convert: openCV image to k4a_image CV_8UC4 \n
  * 
  * input:\n
  * \param	in = cv::Mat image (all formats); format cv image format --- input format 0 = RGBA, 1 = RGB, 2 = Gray; 
  * output:\n
  * \param	out = k4a_image image (RGBA)
  * \return	void
  */
void kv3::convertOpenCvImageToK4aImage(const cv::Mat &in, const int &format, k4a_image_t &out){
    Mat imgRGBA;
    if(format == 0){
        k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, in.cols, in.rows, in.cols * 4 * (int)sizeof(uint8_t), &out);
        memcpy(k4a_image_get_buffer(out), &in.ptr<cv::Vec4b>(0)[0], in.rows * in.cols * sizeof(cv::Vec4b));       
    }else{
        if(format == 1){
            cvtColor(in, imgRGBA, cv::COLOR_RGB2RGBA);
        }
        if(format == 2){
            Mat rgb;
            cvtColor(in, rgb, cv::COLOR_GRAY2RGB);
            cvtColor(rgb, imgRGBA, cv::COLOR_RGB2RGBA);
        }
        k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, in.cols, in.rows, in.cols * 4 * (int)sizeof(uint8_t), &out);
        memcpy(k4a_image_get_buffer(out), &imgRGBA.ptr<cv::Vec4b>(0)[0], imgRGBA.rows * imgRGBA.cols * sizeof(cv::Vec4b));
    }
    //do not forget to  release the k4a_image_t image via "k4a_image_release(k4a_image_t image)".    
}//end_fct
#endif


#ifdef USE_PCL
/**
  * read the point cloud from kv3 sensor and\n
  * store it inside PCL-Lib pointcloud\n
  * input:\n
  * \param	none (internal grab from current kv3 PCL)
  * output:\n
  * \param	PCLlibPointcloud
  * \return	void
  */
void kv3::updatePclforPclLib(pcl::PointCloud<pcl::PointXYZ>::Ptr& outPcl, k4a_image_t k4aPcl){
    int width = k4a_image_get_width_pixels(kv3ImDepth);
    int height = k4a_image_get_height_pixels(kv3ImDepth);    
    uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(kv3ImDepth);
    //cout << "createPclforPclLib - kv3ImDepth x: " << width << " y: " << height << endl;

    k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(kv3_xy_table);
    int widthXYtable = k4a_image_get_width_pixels(kv3_xy_table);
    int heightXYtable = k4a_image_get_height_pixels(kv3_xy_table);
    //cout << "createPclforPclLib - xy_table_data x: " << widthXYtable << " y: " << heightXYtable << endl;

    k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(k4aPcl); //uint 16 ????
    int widthpcld = k4a_image_get_width_pixels(k4aPcl);
    int heightpcld = k4a_image_get_height_pixels(k4aPcl);
    //cout << "createPclforPclLib - k4aPcl x: " << widthpcld << " y: " << heightpcld << endl;

    int maxLimit = width*height;
    int i=0;
    while(i<maxLimit)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
                point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * depth_data[i];
                point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * depth_data[i];
                point_cloud_data[i].xyz.z = depth_data[i];
        }
        else
        {
            point_cloud_data[i].xyz.x = nanf("");
            point_cloud_data[i].xyz.y = nanf("");
            point_cloud_data[i].xyz.z = nanf("");
        }
    i++;
    }

    
    /////////////////////////////////////////////////////////
    // create pcl for PCL library    
    long int counter = 0;
    
    float factor = 0.001;
    float x; float y; float z;
	int j=0;
	int jmax = maxLimit;
	while(j<jmax){
        x = (float)point_cloud_data[j].xyz.x*factor;
        y = (float)point_cloud_data[j].xyz.y*factor;
        z = (float)point_cloud_data[j].xyz.z*factor;
        if(!std::isnan(x)){
            if((x != 0)&&(y != 0)&&(z != 0)){
                pcl::PointXYZ Point = pcl::PointXYZ(x,y,z);
                outPcl->push_back(Point);
                counter++;
            }
        }
	    j++;
	}    
    
    cout << "counter: " << counter << endl;
    outPcl->points.resize((uint32_t)counter);
    
   cout << " ------------------------------- " << endl;
}
#endif

#ifdef USE_PCL
/**
  * read the point cloud from kv3 sensnor and\n
  * store it inside PCL-Lib pointcloud\n
  * input:\n
  * \param	none (internal grab from current kv3 PCL)
  * output:\n
  * \param	RGB-PCLlibPointcloud
  * \return	void
  */
void kv3::updatePclRgbforPclLib(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outPcl){
    //cout << "start ---  updatePclRgbforPclLib " << endl;
    int width = k4a_image_get_width_pixels(kv3ImDepth);
    int height = k4a_image_get_height_pixels(kv3ImDepth);    
    uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(kv3ImDepth);

    k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(kv3_xy_table);
    int widthXYtable = k4a_image_get_width_pixels(kv3_xy_table);
    int heightXYtable = k4a_image_get_height_pixels(kv3_xy_table);

    k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(kv3ImPcl);
    int widthpcld = k4a_image_get_width_pixels(kv3ImPcl);
    int heightpcld = k4a_image_get_height_pixels(kv3ImPcl);

    int point_count = 0;
    int maxLimit = width * height;

    int i=0;
    while(i<maxLimit)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
            point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
            point_cloud_data[i].xyz.z = (float)depth_data[i];
            point_count++;
        }
        else
        {
            point_cloud_data[i].xyz.x = nanf("");
            point_cloud_data[i].xyz.y = nanf("");
            point_cloud_data[i].xyz.z = nanf("");
        }
    i++;
    }

    /////////////////////////////////////////////////////////
    // create RGB pcl for PCL library    
    k4a_image_t kv3ImColorTrans = NULL;
    //COLOR
    uint8_t *pclcolorImg;
    if(kv3ImColor != NULL){        
        k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, width, height, width * 4 * (int)sizeof(uint8_t), &kv3ImColorTrans);
        k4a_transformation_color_image_to_depth_camera(transformation, kv3ImDepth, kv3ImColor, kv3ImColorTrans);
        uint8_t *pclcolorImgBuffer = k4a_image_get_buffer(kv3ImColorTrans);
        pclcolorImg = pclcolorImgBuffer;           
    }
    
    if(outPcl != NULL){
        if(outPcl->points.size() > 0){
            outPcl->clear();
        }
    }
    
    long int counter = 0;
    float factor = 0.001;
    float x; float y; float z;
    uint r; uint g; uint b;
	int j=0;
	int jmax = maxLimit;
	while(j<jmax){
        x = (float)point_cloud_data[j].xyz.x*factor;
        y = (float)point_cloud_data[j].xyz.y*factor;
        z = (float)point_cloud_data[j].xyz.z*factor;
        if(currentSetupId == 4){  
                r = (uint)255;
                g = (uint)255;
                b = (uint)255;
        }else{
            if(kv3ImColor != NULL){
                r = (uint)pclcolorImg[4 * j + 2];
                g = (uint)pclcolorImg[4 * j + 1];
                b = (uint)pclcolorImg[4 * j + 0];
            }else{
                r = (uint)255;
                g = (uint)255;
                b = (uint)255;
            }
        }

        if(!std::isnan(x)){
            if((x != 0)&&(y != 0)&&(z != 0)){
                pcl::PointXYZRGB Point = pcl::PointXYZRGB(x,y,z,r,g,b);
                outPcl->push_back(Point);
                counter++;
            }
        }
        
    j++;
	}    
    outPcl->points.resize((uint32_t)counter);

    k4a_image_release(kv3ImColorTrans);
}
#endif

/**
  * read the point cloud without color data via k4a commands and\n
  * improve the depth values and\n
  * save it as PLY file to give file path.\n
  * input:\n
  * \param	const char *file_name = path and filename to the save location.
  * \return	void
  */
void kv3::writePCL(const char *file_name)
{
    int width = k4a_image_get_width_pixels(kv3ImDepth);
    int height = k4a_image_get_height_pixels(kv3ImDepth);    
    uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(kv3ImDepth);

    k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(kv3_xy_table);
    int widthXYtable = k4a_image_get_width_pixels(kv3_xy_table);
    int heightXYtable = k4a_image_get_height_pixels(kv3_xy_table);

    k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(kv3ImPcl);
    int widthpcld = k4a_image_get_width_pixels(kv3ImPcl);
    int heightpcld = k4a_image_get_height_pixels(kv3ImPcl);

    int point_count = 0;
    
    int maxLimit = width * height;
    //cout << "--- ---- --- maxLimit: " << maxLimit << endl; //ok
    for (int i = 0; i < maxLimit; i++)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
            point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
            point_cloud_data[i].xyz.z = (float)depth_data[i];
            point_count++;
        }
        else
        {
            point_cloud_data[i].xyz.x = nanf("");
            point_cloud_data[i].xyz.y = nanf("");
            point_cloud_data[i].xyz.z = nanf("");
        }
    }

    /////////////////////////////////////////////////////////////////////////
    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << "ply" << std::endl;
    ofs << "format ascii 1.0" << std::endl;
    ofs << "element vertex"
        << " " << point_count << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "element face 0" << std::endl;
    ofs << "property list uchar int vertex_indices" << std::endl;    
    ofs << "end_header" << std::endl;
    ofs.close();

    std::stringstream ss;
    for (int i = 0; i < width * height; i++)
    {
        if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
        {
            continue;
        }

        ss << (float)point_cloud_data[i].xyz.x << " " 
           << (float)point_cloud_data[i].xyz.y << " "
           << (float)point_cloud_data[i].xyz.z << std::endl;
    }

    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}


/**
  * write the point cloud with color data via k4a commands and\n
  * improve the depth values and\n
  * save it as PLY file to give file path.\n
  * input:\n
  * \param	const char *file_name = path and filename to the save location.
  * \return	void
  */
void kv3::writePclRgb(const char *file_name)
{
    int width = k4a_image_get_width_pixels(kv3ImDepth);
    int height = k4a_image_get_height_pixels(kv3ImDepth);    
    uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(kv3ImDepth);

    k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(kv3_xy_table);
    int widthXYtable = k4a_image_get_width_pixels(kv3_xy_table);
    int heightXYtable = k4a_image_get_height_pixels(kv3_xy_table);

    k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(kv3ImPcl);
    int widthpcld = k4a_image_get_width_pixels(kv3ImPcl);
    int heightpcld = k4a_image_get_height_pixels(kv3ImPcl);

    int point_count = 0;
    
    int maxLimit = width * height;
    //cout << "--- ---- --- maxLimit: " << maxLimit << endl; //ok
    for (int i = 0; i < maxLimit; i++)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
            point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
            point_cloud_data[i].xyz.z = (float)depth_data[i];
            point_count++;
        }
        else
        {
            point_cloud_data[i].xyz.x = nanf("");
            point_cloud_data[i].xyz.y = nanf("");
            point_cloud_data[i].xyz.z = nanf("");
        }
    }


    //COLOR
    k4a_image_t kv3ImColorTrans = NULL;
    k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, width, height, width * 4 * (int)sizeof(uint8_t), &kv3ImColorTrans);
    k4a_transformation_color_image_to_depth_camera(transformation, kv3ImDepth, kv3ImColor, kv3ImColorTrans);
    uint8_t *pclcolorImg = k4a_image_get_buffer(kv3ImColorTrans);



    /////////////////////////////////////////////////////////////////////////
    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << "ply" << std::endl;
    ofs << "format ascii 1.0" << std::endl;
    ofs << "element vertex"
        << " " << point_count << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "property uchar red" << std::endl;
    ofs << "property uchar green" << std::endl;
    ofs << "property uchar blue" << std::endl;    
    ofs << "element face 0" << std::endl;
    ofs << "property list uchar int vertex_indices" << std::endl;    
    ofs << "end_header" << std::endl;
    ofs.close();

    std::stringstream ss;
    for (int i = 0; i < width * height; i++)
    {
        if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
        {
            continue;
        }

        ss << (float)point_cloud_data[i].xyz.x << " " 
           << (float)point_cloud_data[i].xyz.y << " "
           << (float)point_cloud_data[i].xyz.z << " "
           << (uint)(pclcolorImg[4 * i + 2])<< " "
           << (uint)(pclcolorImg[4 * i + 1])<< " "
           << (uint)(pclcolorImg[4 * i + 0])<< std::endl;
    }

    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

/**
  * write the point cloud with color data via k4a commands and\n
  * improve the depth values and\n
  * save it as PLY file to give file path.\n
  * input:\n
  * \param	const char *file_name = path and filename to the save location.
  * \return	void
  */
void kv3::writePclRgbAsPCDfile(const char *file_name)
{
    float fc = 0.001;
    int width = k4a_image_get_width_pixels(kv3ImDepth);
    int height = k4a_image_get_height_pixels(kv3ImDepth);    
    uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(kv3ImDepth);

    k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(kv3_xy_table);
    int widthXYtable = k4a_image_get_width_pixels(kv3_xy_table);
    int heightXYtable = k4a_image_get_height_pixels(kv3_xy_table);

    k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(kv3ImPcl);
    int widthpcld = k4a_image_get_width_pixels(kv3ImPcl);
    int heightpcld = k4a_image_get_height_pixels(kv3ImPcl);

    int point_count = 0;
    
    int maxLimit = width * height;
    //cout << "--- ---- --- maxLimit: " << maxLimit << endl; //ok
    for (int i = 0; i < maxLimit; i++)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
            point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
            point_cloud_data[i].xyz.z = (float)depth_data[i];            
        }
        else
        {
            point_cloud_data[i].xyz.x = 0; //nanf("");
            point_cloud_data[i].xyz.y = 0; //nanf("");
            point_cloud_data[i].xyz.z = 0; //nanf("");
        }
        point_count++;
    }


    //COLOR
    k4a_image_t kv3ImColorTrans = NULL;
    k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, width, height, width * 4 * (int)sizeof(uint8_t), &kv3ImColorTrans);
    k4a_transformation_color_image_to_depth_camera(transformation, kv3ImDepth, kv3ImColor, kv3ImColorTrans);
    uint8_t *pclcolorImg = k4a_image_get_buffer(kv3ImColorTrans);

    /////////////////////////////////////////////////////////////////////////
    // save to the pcd file
    std::ofstream ofs(file_name); // text mode first
    ofs << "# .PCD v.7 - Point Cloud Data file format" << std::endl;
    ofs << "VERSION .7" << std::endl;
    ofs << "FIELDS x y z rgb" << std::endl;
    ofs << "SIZE 4 4 4 4" << std::endl;
    ofs << "TYPE F F F F" << std::endl;
    ofs << "COUNT 1 1 1 1 " << std::endl;
    ofs << "WIDTH " << to_string(width) << std::endl;
    ofs << "HEIGHT " << to_string(height)  << std::endl;
    ofs << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    ofs << "POINTS " << to_string(point_count) << std::endl;
    ofs << "DATA ascii" << std::endl;
    ofs.close();   

    std::stringstream ss;
    for (int i = 0; i < width * height; i++)
    {
        if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
        {
            //continue;
            ss << (float)0 << " " 
                << (float)0 << " "
                << (float)0 << " "
                << (float)0 << std::endl;            

        }else{

            uint8_t r = (uint)(pclcolorImg[4 * i + 2]);
            uint8_t g = (uint)(pclcolorImg[4 * i + 1]);
            uint8_t b = (uint)(pclcolorImg[4 * i + 0]);
            float color = (b << 16)|(g << 8)|(r);  
            //float color = (float)uint32Color;
            /*
                var rgb = parseFloat( line[ offset.rgb ] );
                var r = ( rgb >> 16 ) & 0x0000ff;
                var g = ( rgb >> 8 ) & 0x0000ff;
                var b = ( rgb >> 0 ) & 0x0000ff;   
            */

            float x = point_cloud_data[i].xyz.x; 
            float y = point_cloud_data[i].xyz.y; 
            float z = point_cloud_data[i].xyz.z; 
            if( (x < 600)&&(y < 600)&&(z < 600)&&(z > -600) ){
                ss << (float)point_cloud_data[i].xyz.x*fc << " " 
                << (float)point_cloud_data[i].xyz.y*fc << " "
                << (float)point_cloud_data[i].xyz.z*fc << " "
                << (float)color << std::endl;
            }else{
                ss << (float)0 << " " 
                << (float)0 << " "
                << (float)0 << " "
                << (float)0 << std::endl;
            }

        }//end_if_else
    }

    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}


 /**
  * calculate the point cloud with improved the depth values and\n
  * transform color data size to depth image size via k4a commands and\n
  * save it as PLY file to give file path.\n
  * input:\n
  * \param	const char *file_name = path and filename to the save location.
  * \return	void
  */
void kv3::writeColorPcl1(const char *file_name)
{
    int width = k4a_image_get_width_pixels(kv3ImDepth);
    int height = k4a_image_get_height_pixels(kv3ImDepth);    
    uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(kv3ImDepth);

    k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(kv3_xy_table);
    int widthXYtable = k4a_image_get_width_pixels(kv3_xy_table);
    int heightXYtable = k4a_image_get_height_pixels(kv3_xy_table);

    k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)k4a_image_get_buffer(kv3ImPcl);
    int widthpcld = k4a_image_get_width_pixels(kv3ImPcl);
    int heightpcld = k4a_image_get_height_pixels(kv3ImPcl);

    int point_count = 0;
    
    int maxLimit = width * height;
    //cout << "--- ---- --- maxLimit: " << maxLimit << endl; //ok
    for (int i = 0; i < maxLimit; i++)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
            point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
            point_cloud_data[i].xyz.z = (float)depth_data[i];
            point_count++;
        }
        else
        {
            point_cloud_data[i].xyz.x = nanf("");
            point_cloud_data[i].xyz.y = nanf("");
            point_cloud_data[i].xyz.z = nanf("");
        }
    }
    /////////////////////////////////////////////////////////////////////////////////////
    // color image resize
    k4a_image_t kv3ImColorTrans = NULL;
    k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, width, height, width * 4 * (int)sizeof(uint8_t), &kv3ImColorTrans);
    k4a_transformation_color_image_to_depth_camera(transformation, kv3ImDepth, kv3ImColor, kv3ImColorTrans);
    uint8_t *colorImg = k4a_image_get_buffer(kv3ImColorTrans);

    
    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << "ply" << std::endl;
    ofs << "format ascii 1.0" << std::endl;
    ofs << "element vertex"
        << " " << point_count << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "property uchar red" << std::endl;
    ofs << "property uchar green" << std::endl;
    ofs << "property uchar blue" << std::endl;
    ofs << "element face 0" << std::endl;
    ofs << "property list uchar int vertex_indices" << std::endl;    
    ofs << "end_header" << std::endl;
    ofs.close();

    std::stringstream ss;
    for (int i = 0; i < width * height; i++)
    {
        if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
        {
            continue;
        }

        ss << (float)point_cloud_data[i].xyz.x << " " 
        << (float)point_cloud_data[i].xyz.y << " "
        << (float)point_cloud_data[i].xyz.z << " "
        << (uint)colorImg[4 * i + 2] << " "
        << (uint)colorImg[4 * i + 1] << " "
        << (uint)colorImg[4 * i + 0] << " "
        << std::endl;
    }

    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}