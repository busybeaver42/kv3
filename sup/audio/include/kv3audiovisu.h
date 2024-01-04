/*
 * kv3audioVisu.h
 *
 *  Created on: 09.03.2023
 *      Author: joerg angermayer
 *     Licence: MIT
 */
/**
* @file kv3audioVisu.h
* @brief Implementation of the class functions to visualize the kinect azure microphone audio data.
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
#ifndef KV3_AUDIOVISU_H_
#define KV3_AUDIOVISU_H_

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
#include <iomanip>
using namespace std;

#ifdef USE_CV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
using namespace cv;
#endif
//#include "kv3audio.h"


class kv3audiovisu {
	public:
	kv3audiovisu();
	~kv3audiovisu();
	#ifdef USE_CV
	cv::Mat drawOdasTrackingData(int idx, float x, float y, float z);
	#endif
	/*
	void showMicrophone();	
	void kv3MicroPrintData(kv3audio kv3audioObj);
	int kv3ShowAudioGraph(kv3audio &kv3audioObj);
	int kv3MicroShowWaveGraph(kv3audio &kv3audioObj);
	*/
	
};
#endif
