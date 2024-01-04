//============================================================================
// Name        : kv3audiovisu.cpp
// Author      : Joerg Angermayer
// Version     : 1.0.0
// Copyright   : MIT
// Description : C++, Ansi-style
//============================================================================
/**
 * @file  kv3audiovisu.cpp
 * @brief Implementation of the class functions to visualize the kinect azure microphone audio data.
 * @brief created on:  09.02.2023;  last update: 24.06.2023
 * @author Jörg Angermayer\n
 * @copyright Licensed under MIT
 */
#include "kv3audiovisu.h"

kv3audiovisu::kv3audiovisu() {}
kv3audiovisu::~kv3audiovisu() {}

#ifdef USE_ODAS
#ifdef USE_CV
cv::Mat kv3audiovisu::drawOdasTrackingData(int idxin, float xin, float yin, float zin){
	//int retval;
	cv::Mat visuOdasTrack(400, 400, CV_8UC3,Scalar(160, 160, 160));
	
	visuOdasTrack = cv::Mat::zeros( visuOdasTrack.size(), visuOdasTrack.type() );
	const int thickness = 4;
	const float factor = 100;
	const int xOffset = (int)(visuOdasTrack.size().width/2);
	int yOffset = (int)(visuOdasTrack.size().height/3);

		// X + Y
		int x = (int)round((float)xin * (float)factor);
		int y = (int)round((float)yin * (float)factor);
		Point p01(xOffset, yOffset), p02(xOffset+y, yOffset+x); // x=y and y=x
		line(visuOdasTrack, p01, p02, Scalar(150, 250, 150), thickness, LINE_8);

		// Z
		int yyOffset = yOffset + 260;
		//ground line
		Point p03(xOffset-100, yyOffset), p04(xOffset+100, yyOffset);
		line(visuOdasTrack, p03, p04, Scalar(200, 200, 200), thickness, LINE_8);

		int z = (int)round((float)zin * (float)factor);
		Point p05(xOffset, yyOffset), p06(xOffset, yyOffset-z);
		line(visuOdasTrack, p05, p06, Scalar(150, 250, 150), thickness, LINE_8);

		circle(visuOdasTrack, p01, factor, Scalar(200, 200, 200), 1);

		
		//write text
		const float fontScale = 0.5;
		const int txtOffset = 20;
		cv::putText(visuOdasTrack, 
            "Front", //text
            cv::Point(xOffset-txtOffset, 20),
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
            1);

		cv::putText(visuOdasTrack, 
            "Rear", //text
            cv::Point(xOffset-txtOffset, yOffset+yOffset-10), 
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
            1);			

		cv::putText(visuOdasTrack, 
            "Left", //text
            cv::Point(xOffset-150-txtOffset, yOffset), 
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
            1);	

		cv::putText(visuOdasTrack, 
            "Right", //text
            cv::Point(xOffset+150-txtOffset, yOffset),
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
            1);	

			//Z
			std::stringstream streamZ;
			streamZ << std::fixed << std::setprecision(3) << zin;
			string strZvalue = string("Z: ") + streamZ.str();
			cv::putText(visuOdasTrack, 
            strZvalue.c_str(), //text
            cv::Point(xOffset-txtOffset, yyOffset-z-10), 
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
            1);	

			// IDX
			string strIdxValue = string("Idx: ") + std::to_string(idxin);
			cv::putText(visuOdasTrack, 
            strIdxValue.c_str(), //text
            cv::Point(20, yOffset+yOffset+20),
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0), 
            1);					
			//X
			std::stringstream streamX;
			streamX << std::fixed << std::setprecision(3) << xin;
			string strXvalue = string("X: ") + streamX.str();
			cv::putText(visuOdasTrack, 
            strXvalue.c_str(), //text
            cv::Point(20, yOffset+yOffset+40), 
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
            1);	
			//Y
			std::stringstream streamY;
			streamY << std::fixed << std::setprecision(3) << yin;
			string strYvalue = string("Y: ") + streamY.str();
			cv::putText(visuOdasTrack, 
            strYvalue.c_str(), //text
            cv::Point(20, yOffset+yOffset+60),
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
            1);					

			// IDX
			cv::putText(visuOdasTrack, 
            strIdxValue.c_str(), //text
            cv::Point(xOffset+y+6, yOffset+x-6),
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            CV_RGB(118, 185, 0),
            1);					
	
return(visuOdasTrack);
}
#endif
#endif
/*
void kv3audiovisu::showMicrophone(){
	cout << "------------------- start kv3Micro... "<< endl;
	bool doMicRecToFile = false;
	//audio
	kv3audio kv3audioObj = kv3audio();
	kv3audioObj.setRecordMicroToFile(doMicRecToFile, "/var/www/ramdev/kv3.wav");
	kv3audioObj.setAudioDevice("plughw:2,0"); // plughw:card,device
	//kv3audioObj.setAudioRate((unsigned int)16000);
	kv3audioObj.setChannels((unsigned int)7);//7
	cout << "------------------- start kv3Micro loop... " << endl;
	while(1){
		kv3audioObj.run();
		kv3MicroPrintData(kv3audioObj);

		if(doMicRecToFile == false){
			//if(kv3MicroShowWaveGraph(kv3audioObj) == 1){break;};
			if(kv3ShowAudioGraph(kv3audioObj) == 1){break;};			
		}//end_doMicRecToFile

	}//end_while
}

void kv3audiovisu::kv3MicroPrintData(kv3audio kv3audioObj){
	int frameSize = kv3audioObj.getFrameByteSize();
	int periodeSize = kv3audioObj.getPeriodeByteSize();
	int sampleSize = kv3audioObj.getSampleByteSize();
	cout << " periodeSize: " << periodeSize << " frameSize: " << frameSize << " sampleSize: " << sampleSize << endl;	
}

int kv3audiovisu::kv3ShowAudioGraph(kv3audio &kv3audioObj){
	int retval;	
	int thickness = 1;
	int i = 0;
	int x = 0;
	int xmax = 0;
	int y = 0;
	int audioOut = 0;
	int visH = 200;
	int visW = 1378;
	const int yOffset = -100;
	float factor = ((float)visH / (float)4294967296); // (float)32768);
	cv::Mat ch01Vis01(visH, visW, CV_8UC3,Scalar(160, 160, 160));
	cv::Mat ch01Vis02(visH, visW, CV_8UC3,Scalar(160, 160, 160));
	cv::Mat ch01Vis03(visH, visW, CV_8UC3,Scalar(160, 160, 160));
	cv::Mat ch01Vis04(visH, visW, CV_8UC3,Scalar(160, 160, 160));

	ch01Vis01 = cv::Mat::zeros( ch01Vis01.size(), ch01Vis01.type() );
	ch01Vis02 = cv::Mat::zeros( ch01Vis02.size(), ch01Vis02.type() );
	ch01Vis03 = cv::Mat::zeros( ch01Vis03.size(), ch01Vis03.type() );
	ch01Vis04 = cv::Mat::zeros( ch01Vis04.size(), ch01Vis04.type() );

	//ch1 part 1
	i = 0;
	x = 0;
	xmax = visW;
	while(x < xmax){
		kv3audioObj.getFrameBuffer(i, 0, audioOut);
		y = (int)round((float)audioOut * (float)factor);
		Point ch0p11(x, visH+yOffset), ch0p12(x, visH-y+yOffset);
		line(ch01Vis01, ch0p11, ch0p12, Scalar(255, 0, 0), thickness, LINE_8);
		i++;
	x++;
	}

	//ch1 part 2
	i = visW;
	x = 0;
	xmax = visW;
	while(x < xmax){
		kv3audioObj.getFrameBuffer(i, 0, audioOut);
		y = (int)round((float)audioOut * (float)factor);
		Point ch0p21(x, visH+yOffset), ch0p22(x, visH-y+yOffset);
		line(ch01Vis02, ch0p21, ch0p22, Scalar(255, 0, 0), thickness, LINE_8);
		i++;
	x++;
	}

	//ch1 part 3
	i = visW*2;
	x = 0;
	xmax = visW;
	while(x < xmax){
		kv3audioObj.getFrameBuffer(i, 0, audioOut);
		y = (int)round((float)audioOut * (float)factor);
		Point ch0p31(x, visH+yOffset), ch0p32(x, visH-y+yOffset);
		line(ch01Vis03, ch0p31, ch0p32, Scalar(255, 0, 0), thickness, LINE_8);
		i++;
	x++;
	}

	//ch1 part 4
	i = visW*3;
	x = 0;
	xmax = visW;
	while(x < xmax){
		kv3audioObj.getFrameBuffer(i, 0, audioOut);
		y = (int)round((float)audioOut * (float)factor);
		Point ch0p41(x, visH+yOffset), ch0p42(x, visH-y+yOffset);
		line(ch01Vis04, ch0p41, ch0p42, Scalar(255, 0, 0), thickness, LINE_8);
		i++;
	x++;
	}		
	
	imshow("ch01Vis01",ch01Vis01);
	imshow("ch01Vis02",ch01Vis02);
	imshow("ch01Vis03",ch01Vis03);
	imshow("ch01Vis04",ch01Vis04);	
	char key = waitKey(1);
	if(key == 27){ retval = 1; }else{retval = 0;}
return(retval);
}

int kv3audiovisu::kv3MicroShowWaveGraph(kv3audio &kv3audioObj){
	int retval;
	int step = 2;
	int visuAudioH = 200;
	int visuAudioW = 3310;//5512;
	float factor = ((float)visuAudioH / (float)65000);
	int audioOut = 0;	
	cv::Mat visuAudioCh0(visuAudioH, visuAudioW, CV_8UC3,Scalar(160, 160, 160));
	cv::Mat visuAudioCh1(visuAudioH, visuAudioW, CV_8UC3,Scalar(160, 160, 160));
	int thickness = 1;

	visuAudioCh0 = cv::Mat::zeros( visuAudioCh0.size(), visuAudioCh0.type() );
	visuAudioCh1 = cv::Mat::zeros( visuAudioCh0.size(), visuAudioCh0.type() );

	int range = visuAudioW;
	int offset = kv3audioObj.getFrameByteSize() - range;//4412;
	int i = offset+1;
	int xmax = range;//max 5512

	int x = 0;
	int y;
	while(x < xmax){
		kv3audioObj.getFrameBuffer(i, 0, audioOut);
		
		//ch0				
		y = (int)round((float)audioOut * (float)factor);
		//cout << "y = " << y << endl;
		Point ch0p1(x, visuAudioH), ch0p2(x, visuAudioH-y);
		line(visuAudioCh0, ch0p1, ch0p2, Scalar(255, 0, 0), thickness, LINE_8);

		//ch1
		kv3audioObj.getFrameBuffer(i, 1, audioOut);
		y = (int)round((float)audioOut * (float)factor);
		Point ch1p1(x, visuAudioH), ch1p2(x, visuAudioH-y);
		line(visuAudioCh1, ch1p1, ch1p2, Scalar(255, 0, 0), thickness, LINE_8);

	x++;
	i = i+1;
	if(i >= 5512){ i = 5512-1; }

	}

	imshow("visuAudioCh0",visuAudioCh0);
	imshow("visuAudioCh1",visuAudioCh1);
	char key = waitKey(1);
	if(key == 27){ retval = 1; }else{retval = 0;}

	return(retval);
}
*/