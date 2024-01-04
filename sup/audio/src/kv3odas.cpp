//============================================================================
// Name        : kv3odas.cpp
// Author      : Joerg Angermayer
// Version     : 1.0.0
// Copyright   : MIT
// Description : C++, Ansi-style
//============================================================================
#ifdef USE_ODAS
/**
 * @file  kv3odas.cpp
 * @brief Implementation of the class functions to get from ODAS server the position data of active audio source/s.
 * @brief created on:  09.02.2023;  last update: 24.06.2023
 * @author Jörg Angermayer\n
 * @copyright Licensed under MIT
 */
#include "kv3odas.h"

    #include <stdlib.h>
    #include <stdio.h>
    #include <string.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <signal.h>
    #include <getopt.h>

kv3odas::kv3odas() {
    kv3odas::kv3audiovisuObj = kv3audiovisu();
}
kv3odas::kv3odas(unsigned int port){
    kv3odas::kv3audiovisuObj = kv3audiovisu();
    kv3odas::portNumber = port;
    //cout << "cd /media/hd2/git_pool_hd2/ODASpool/odas/build/bin" << endl;
    //cout << "sudo ./odaslive -vc /media/hd2/git_pool_hd2/ODASpool/odas/config/odaslive/kv3Socket.cfg" << endl;
}

kv3odas::~kv3odas() {}

unsigned int kv3odas::portNumber = 9000;
string kv3odas::strFilename = string("/var/www/ramdev/odas.sock"); //target to create the socket
string kv3odas::strOdasLivePath = string("/media/hd2/git_pool_hd2/ODASpool/odas/build/bin");
tSoundStruct kv3odas::soundObj[];
kv3audiovisu kv3odas::kv3audiovisuObj;

string kv3odas::getFilename(){return(kv3odas::strFilename);}
unsigned int kv3odas::getPortNumber(){return(kv3odas::portNumber);}
void kv3odas::setFilename(string str){ kv3odas::strFilename = str;}
void kv3odas::setPortNumber(unsigned int port){ kv3odas::portNumber = port;}

 /**
  * grab data from ODAS server
  * input:\n
  * \param	int index of data, string of data 
  * \return	void
  *
  */
void kv3odas::grabData(int index, string str){
    //cout << "index: " << index << " str: " << str << endl;
    int idPos = str.find("\"tag\"");
    string strId = str.substr(0, idPos-2);
    try {
        kv3odas::soundObj[index].id = (int)std::stoi(strId);
    } 
    catch (exception e) {
        //cout << "string not converted";
    }

    int xpos = str.find("x");
    string strX = str.substr(xpos+3, str.size());
    strX = strX.substr(0, 7);

    int ypos = str.find(" \"y\":");
    string strY = str.substr(ypos+5, str.size());
    strY = strY.substr(0, 7);    

    int zpos = str.find("z");
    string strZ = str.substr(zpos+3, str.size());
    strZ = strZ.substr(0, 7);  

    try {
        kv3odas::soundObj[index].x = (float)std::stof(strX);
    } 
    catch (exception e) {
        //cout << "--- ERROR --- stof X --- strX:" << strX;
    }    

    try {
        kv3odas::soundObj[index].y = (float)std::stof(strY);

    } 
    catch (exception e) {
        //cout << "--- ERROR --- stof X --- strX:" << strY;
    }  

    try {
        kv3odas::soundObj[index].z = (float)std::stof(strZ);
    } 
    catch (exception e) {
        //cout << "--- ERROR --- stof X --- strX:" << strZ;
    }  

    //string strX = str.substr(xpos, xEndPos);
    //cout << "index: " << index << " strX: " << strX << " strY: " << strY << " strZ: " << strZ << endl;
    //cout << "index: " << index << " strX: " << kv3odas::soundObj[index].x << " strY: " << kv3odas::soundObj[index].y << " strZ: " << kv3odas::soundObj[index].z << endl;
}

 /**
  * execute ODAS command
  * input:\n
  * \param	string command
  * \return	String result
  *
  */

string kv3odas::exec(string cmd){
   char buffer[128];
   string result = "";

   // Open pipe to file
   FILE* pipe = popen(cmd.c_str(), "r");
   if (!pipe) {
      return "popen failed!";
   }

   // read till end of process:
   while (!feof(pipe)) {

      // use buffer to read and add to result
      if (fgets(buffer, 128, pipe) != NULL)
         result += buffer;
   }

   pclose(pipe);
   return result;
}

 /**
  * Start ODAS
  * \return	void
  *
  */
void kv3odas::startOdasLive(){
    string cmd01 = string("cd ") + kv3odas::strOdasLivePath;
    //std::system(cmd01.c_str());
    string cmd02 = string("arecord -l");
    //string cmd03 = string("sudo ./odaslive -vc /media/hd2/git_pool_hd2/ODASpool/odas/config/odaslive/kv3Socket.cfg");

    //cmd02
    std::string strResult = exec(cmd02);
    cout << " strResult: " << strResult << endl;
    int indexFindResult = strResult.find("Azure Kinect Microphone Array");
    if(indexFindResult >= 0){
        int start = indexFindResult - 16;
        int end = indexFindResult + 40;
        int maxChara = end - start;
        string strToProof = strResult.substr(start, maxChara);
        //cout << "strToProof: " << strToProof << endl;
        int indexCard = strToProof.find("Karte");
        int indexDevice = strToProof.find("Gerät");
        int soundCardId = std::stoi(strToProof.substr(indexCard+6, 1));    
        int soundDeviceId = std::stoi(strToProof.substr(indexDevice+7, 1));
        cout << "Azure Kinect Microphone Array --- soundCardId: " << soundCardId << " soundDeviceId: " << soundDeviceId << endl;

        string strPathToCfg = string("/media/hd2/dev/kinectV3/kv3/cfg/");
        string strOdasCfgFile = string("");
        if(soundCardId == 0){            
            strOdasCfgFile = string("kv3OdasSocketCard0.cfg");            
        }
        if(soundCardId == 1){            
            strOdasCfgFile = string("kv3OdasSocketCard1.cfg");            
        }
        if(soundCardId == 2){            
            strOdasCfgFile = string("kv3OdasSocketCard2.cfg");            
        }        
        string cmd03 = string("sudo ./odaslive -vc ") + strPathToCfg + strOdasCfgFile;


        //std::system(cmd01.c_str());        
        //cout << "cmd01.c_str(): \n" << cmd01.c_str() << endl;
        //std::system("ls"); 
        //std::system("pwd"); 
        int result;
        cout << "---------------------------------------" << endl;
        result = std::system(cmd01.c_str());
        cout << "cmd01: result: " << result << " CMD: " << cmd01 << endl;
        result = std::system(cmd03.c_str());
        cout << "cmd03: result: " << result << " CMD: " << cmd03 << endl;

    }else{
        cout << "Azure Kinect Microphone Array not detected - proof the hardware connection, please." << endl;
    }
}

 /**
  * start up connection to ODAS server
  * \return	void
  *
  */
void kv3odas::odasServer(){
        char verbose = 0x00;
        int server_id;
        struct sockaddr_in server_address;
        int connection_id;
        char * message;
        int messageSize;

        int c;
        //unsigned int portNumber = 9000;
        FILE * fp;

        const unsigned int nBytes = 10240;

        server_id = socket(AF_INET, SOCK_STREAM, 0);

        server_address.sin_family = AF_INET;
        server_address.sin_addr.s_addr = htonl(INADDR_ANY);
        server_address.sin_port = htons(kv3odas::portNumber);

        printf("Opening file............. "); fflush(stdout);
        fp = fopen(strFilename.c_str(), "wb");
        
        printf("[OK]\n");

        printf("Binding socket........... ");  fflush(stdout);
        bind(server_id, (struct sockaddr *) &server_address, sizeof(server_address));
        printf("[OK]\n");

        printf("Listening socket......... ");  fflush(stdout);
        listen(server_id, 1);
        printf("[OK]\n");

        printf("Waiting for connection... "); fflush(stdout);
        connection_id = accept(server_id, (struct sockaddr*) NULL, NULL);
        printf("[OK]\n");

        message = (char *) malloc(sizeof(char) * nBytes);

        printf("Receiving data........... "); fflush(stdout);
        while( (messageSize = recv(connection_id, message, nBytes, 0)) > 0) {

            message[messageSize] = 0x00;
            //fwrite(message, messageSize, sizeof(char), fp);
            string odasMsg = string(message);
            odasMsg = odasMsg.substr(0, odasMsg.size()-11);
            int posId1 = odasMsg.find("\"src\":");            
            string odasMsgcore = odasMsg.substr(posId1+9, odasMsg.size());
            ///////////////////////////////////////////////////////
 
            int umbruch1 = odasMsgcore.find("\n");
            string odasData01 = odasMsgcore.substr(15, umbruch1-18);
            grabData(0, odasData01);
            string odasMsgcore2 = odasMsgcore.substr(umbruch1+1, odasMsgcore.size());

            int umbruch2 = odasMsgcore2.find("\n");
            string odasData02 = odasMsgcore.substr(umbruch1+16, umbruch2-18);
            grabData(1, odasData02);
            string odasMsgcore3 = odasMsgcore.substr(umbruch2+1, odasMsgcore2.size());
            
            int umbruch3 = odasMsgcore3.find("\n");
            string odasData03 = odasMsgcore3.substr(umbruch2+16, umbruch3-18);            
            grabData(2, odasData03);
            string odasMsgcore4 = odasMsgcore3.substr(umbruch3+1, odasMsgcore3.size());

            int umbruch4 = odasMsgcore4.find("\n");
            string odasData04 = odasMsgcore4.substr(umbruch3+16, umbruch4-18);
            grabData(3, odasData04);

            //int sid = 0;
            /*
            int idx;
            float x;
            float y;
            float z;
            
            float len;
            getNearesSoundObj(idx, x, y, z, len);
            */
            //cout << "idx: " << idx << " x: " << x << " y: " << y << " z: " << z << " len: " << len << endl;
   //         if(kv3audiovisuObj.drawOdasTrackingData(idx,x,y,z,"SoundSourceLocation") == 1){break;}
            /*
            this->getSoundObj(0, idx, x, y, z);
            if(kv3audiovisuObj.drawOdasTrackingData(idx,x,y,z,"view01") == 1){break;}

            this->getSoundObj(1, idx, x, y, z);
            if(kv3audiovisuObj.drawOdasTrackingData(idx,x,y,z,"view02") == 1){break;}

            this->getSoundObj(2, idx, x, y, z);
            if(kv3audiovisuObj.drawOdasTrackingData(idx,x,y,z,"view03") == 1){break;}

            this->getSoundObj(3, idx, x, y, z);
            if(kv3audiovisuObj.drawOdasTrackingData(idx,x,y,z,"view04") == 1){break;}
            */
        }
        printf("[OK]\n");

        printf("Closing file............. "); fflush(stdout);
        fclose(fp);
        printf("[OK]\n");
}

 /**
  * grab the id from the nearest sound source
  * output:\n
  * \param	int id, float x, float y, float z, float len
  * \return	void
  *
  */
void kv3odas::getNearesSoundObj(int &idx, float &x, float &y, float &z, float &len){
    int maxId = 0;
    float formerLen = 0;
    int id = 0;
    while(id < 4){
        float lenInput = (kv3odas::soundObj[id].x*kv3odas::soundObj[id].x) + (kv3odas::soundObj[id].y*kv3odas::soundObj[id].y) + (kv3odas::soundObj[id].z*kv3odas::soundObj[id].z);
        float len = std::sqrt(lenInput);
        kv3odas::soundObj[id].len = len;
        if(len > formerLen){ maxId = id;}
        formerLen = len;
        id++;
    }
    if(soundObj!=NULL){
        x = kv3odas::soundObj[maxId].x;
        y = kv3odas::soundObj[maxId].y;
        z = kv3odas::soundObj[maxId].z;
        idx = kv3odas::soundObj[maxId].id;
        len = kv3odas::soundObj[maxId].len;
    }else{
        x = 0;
        y = 0;
        y = 0;
        idx = -1;
        len = 0;
    }
    
}

 /**
  * get data from ODAS sound object
  * input:\n
  * \param	int sound id  
  * output:\n
  * \param int idx, float x, float y, float z
  * \return	void
  *
  */
void kv3odas::getSoundObj(int sid, int &idx, float &x, float &y, float &z){
    x = kv3odas::soundObj[sid].x;
    y = kv3odas::soundObj[sid].y;
    z = kv3odas::soundObj[sid].z;
    idx = kv3odas::soundObj[sid].id;
}

 /**
  * set ODAS Live path
  * input:\n
  * \param	string path to ODAS live server 
  * \return	void
  *
  */
void kv3odas::setOdasLivePath(string str){
    kv3odas::strOdasLivePath = str;
}

 /**
  * draw ODAS tracking data
  * input:\n
  * \param	int sound id, float x, float y, float z  
  * \return	cv::Mat visualisation picture
  *
  */
cv::Mat kv3odas::drawOdasTrackingData(int idxin, float xin, float yin, float zin){
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