/*
 * kv3odas.h
 *
 *  Created on: 09.03.2023
 *      Author: joerg angermayer
 *     Licence: MIT
 */
/**
* @file kv3odas.h
* @brief Implementation of the class functions to get from ODAS server the position data of active audio source/s.
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
#include <stdexcept>
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
using namespace std;



#ifndef KV3_ODAS_H_
#define KV3_ODAS_H_

#include "kv3audiovisu.h"
//#include "kv3.h"

typedef struct {
	int id;
	string tag;
	float x;
	float y;
	float z;
	float activity;
	float len;
}tSoundStruct;

class kv3odas {
	public:
		kv3odas();
		kv3odas(unsigned int port);
		//kv3odas(unsigned int port, kv3 &kv3Obj);
		~kv3odas();
		void startOdasLive();
        void odasServer();
		string getFilename();
		unsigned int getPortNumber();
		void setFilename(string str);
		void setPortNumber(unsigned int port);
		void setOdasLivePath(string str);
		void getNearesSoundObj(int &id, float &x, float &y, float &z, float &len);
		void getSoundObj(int sid, int &idx, float &x, float &y, float &z);
		cv::Mat drawOdasTrackingData(int idxin, float xin, float yin, float zin);
	private:
		void grabData(int index,string str);
		string exec(string cmd);
		static unsigned int portNumber;
		static string strFilename;
		static string strOdasLivePath;
		static tSoundStruct soundObj[4];		
		static kv3audiovisu kv3audiovisuObj;		
};
#endif
