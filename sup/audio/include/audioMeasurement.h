#ifndef AUDIO_MEASUREMENT_H_
#define AUDIO_MEASUREMENT_H_

#define MAX_PVALUE_X_BUFFER 800
#define MAX_PVALUE_Y_BUFFER 200

#include<alsa/asoundlib.h>
#include<math.h>

#include <iostream>
#include <thread>
#include <fstream>
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
#include <iomanip> //setprecision
using namespace std;

#ifdef USE_CV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
using namespace cv;
#endif

class audioMeasurement {
	public:
		audioMeasurement();
		~audioMeasurement();		
		void run();
		void update();
		double rms(short *buffer);
		void show(int Pvalue, int peak);
		float getPvalue();
		float getPvalueMax();
		int getIntPvalue();
		int getIntPvalueMax();
		#ifdef USE_CV
		cv::Mat getMeasurementImage();
		#endif	

	private:
		void init();
		#ifdef USE_CV
		void updateImage();
		#endif
		static char *device;
		static short buffer[8*1024];
		static int buffer_size;
		static snd_pcm_t *handle_capture;
    	static snd_pcm_sframes_t frames;
		static double k;
		static double Pvalue;
		static double peak;
		int err;  
		static int counterpValueBuffer;
		static int pValueBuffer[MAX_PVALUE_X_BUFFER];
		#ifdef USE_CV
		static cv::Mat audioMeasurementImg;
		#endif		
};
#endif /* AUDIO_MEASUREMENT_H_*/