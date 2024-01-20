/*
 * pclUtilitys.cpp
 *
 *  Created on: 14.01.2023
 *      Author: joerg
 */
/**
 * @file  pclUtilitys.cpp
 * @brief Implementation of the class functions to support the user with usefull functions.
 * @brief created on:  09.02.2023;  last update: 24.06.2023
 * @author Jörg Angermayer\n
 * @copyright Licensed under MIT
 */
#ifdef USE_PCL
#include "pclUtilitys.h"

int pclUtilitys::rootPclSize;
int pclUtilitys::rootPclRgbSize;
int pclUtilitys::rootPclResultSize;
int pclUtilitys::rootPclRgbResultSize;
pcl::search::KdTree<pcl::PointXYZ>::Ptr pclUtilitys::tree_xyz (new pcl::search::KdTree<pcl::PointXYZ>());
pcl::search::KdTree<pcl::PointXYZRGB>::Ptr pclUtilitys::tree_xyzrgb (new pcl::search::KdTree<pcl::PointXYZRGB>());

pclUtilitys::pclUtilitys() {
	 //pcl::visualization::PCLVisualizer::Ptr initViewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	 pcl::visualization::PCLVisualizer::Ptr initViewer;
	 pclUtilitys::viewer = initViewer;
}

pclUtilitys::~pclUtilitys() {
}

void pclUtilitys::getBiggestPlaneLocation(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointXYZ& out){
	//better implement it in opencv, because of of the cv::mat handling
	//proof gradienten grid

	//proof the whole cloud 

}

 /**
  * set wall relative to detected origin XY
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>&
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::setWallrelativeToOriginXY(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rawWallPlanePCL (new pcl::PointCloud<pcl::PointXYZRGB>);
	const float range = 0.4;
	float minX = range * (-1);
	float maxX = range;
	float minY = range * (-1);
	float maxY = range;	
	int i=0;
	int imax = in.points.size();
	while(i<imax){
		if((in.points[i].x > minX)&&(in.points[i].x < maxX)&&
		(in.points[i].y > minY)&&(in.points[i].y < maxY)
		){
			pcl::PointXYZRGB bufferPoint;
			bufferPoint.x = in.points[i].x;
			bufferPoint.y = in.points[i].y;
			bufferPoint.z = in.points[i].z;
			bufferPoint.r = in.points[i].r;
			bufferPoint.g = in.points[i].g;
			bufferPoint.b = in.points[i].b;
			rawWallPlanePCL->push_back(bufferPoint);
		}
		i++;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr wallPlaneFil (new pcl::PointCloud<pcl::PointXYZ>);
	convertPtrXYZRGBtoXYZ(*rawWallPlanePCL, wallPlaneFil);
	//radiusOutlierRemovalhelpFilter(*wallPlaneFil, *wallPlaneFil, 0, 0.22, 600);
	//radiusOutlierRemovalhelpFilter(*wallPlaneFil, *wallPlaneFil, 0, 0.4, 300);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outPcl (new pcl::PointCloud<pcl::PointXYZRGB>);
	this->convertPtrXYZtoXYZRGB(*wallPlaneFil, outPcl);
	extractPlaneFromCloudRgb(*outPcl, *outPcl, false);

	//////////////////////get distance
	float deltaYup = 0.2;
	float rangeXLimit1 = 0.05;
	float rangeXLimit2 = 0.15;
	float rangeYLimit1 = 0.12;
	float rangeYLimit2 = 0.18;	
	float rangeY2Limit1 = rangeYLimit1 + deltaYup;
	float rangeY2Limit2 = rangeYLimit2 + deltaYup;
	//side A
	double summSideA = 0;
	long int summSideACounter = 0;
	double summSideB = 0;
	long int summSideBCounter = 0;
	double summSideC = 0;
	long int summSideCCounter = 0;	
	int j=0;
	int jmax=outPcl->points.size();
	while(j<jmax){
		outPcl->points[j].r = 160;
		outPcl->points[j].g = 160;
		outPcl->points[j].b = 160;

		//side A
		if((outPcl->points[j].x > rangeXLimit1)&&(outPcl->points[j].x < rangeXLimit2)&&
		   (outPcl->points[j].y > rangeYLimit1)&&(outPcl->points[j].y < rangeYLimit2)
		){
			summSideA = summSideA + outPcl->points[j].z;
			summSideACounter++;
			outPcl->points[j].r = 255;
			outPcl->points[j].g = 0;
			outPcl->points[j].b = 0;
		}
		//side B
		if((outPcl->points[j].x > -rangeXLimit2)&&(outPcl->points[j].x < -rangeXLimit1)&&
		   (outPcl->points[j].y > rangeYLimit1)&&(outPcl->points[j].y < rangeYLimit2)
		){
			summSideB = summSideB + outPcl->points[j].z;
			summSideBCounter++;
			outPcl->points[j].r = 0;
			outPcl->points[j].g = 255;
			outPcl->points[j].b = 0;			
		}		
		//upper side
		if((outPcl->points[j].x > -rangeXLimit2)&&(outPcl->points[j].x < -rangeXLimit1)&&
		   (outPcl->points[j].y > rangeY2Limit1)&&(outPcl->points[j].y < rangeY2Limit2)
		){
			summSideC = summSideC + outPcl->points[j].z;
			summSideCCounter++;
			outPcl->points[j].r = 0;
			outPcl->points[j].g = 0;
			outPcl->points[j].b = 255;			
		}		
		if(j==jmax-1){
			//cout << "check" << endl;
			if(summSideACounter == 0){
				cout << "Counter A error ..." << endl;
				cout << " rangeXLimit1:" << rangeXLimit1 << " rangeXLimit2:" << rangeXLimit2 << endl;
				cout << " rangeYLimit1:" << rangeYLimit1 << " rangeYLimit2:" << rangeYLimit2 << endl;	
				rangeXLimit1 = rangeXLimit1 + 0.01;
				rangeXLimit2 = rangeXLimit2 + 0.01;
				if(rangeXLimit1 > 10){
					rangeXLimit1 = rangeXLimit1 - 10;
					rangeXLimit2 = rangeXLimit2 - 10;
				}

				//rangeYLimit1 = rangeYLimit1 - 0.01;
				//rangeYLimit2 = rangeYLimit2 - 0.01;
				if(rangeYLimit1 > 10){j = jmax;}
				j = 0;
			}
			if(summSideBCounter == 0){
				cout << "Counter B error ..." << endl;
				cout << " rangeXLimit1:" << rangeXLimit1 << " rangeXLimit2:" << rangeXLimit2 << endl;
				cout << " rangeYLimit1:" << rangeYLimit1 << " rangeYLimit2:" << rangeYLimit2 << endl;				
				rangeYLimit1 = rangeYLimit1 - 0.01;
				rangeYLimit2 = rangeYLimit2 - 0.01;
				if(rangeYLimit1 > 10){j = jmax;}
				j = 0;
			}			
			if(summSideBCounter == 0){
				cout << "Counter C error ..." << endl;
				cout << " rangeXLimit1:" << rangeXLimit1 << " rangeXLimit2:" << rangeXLimit2 << endl;
				cout << " rangeYLimit1:" << rangeYLimit1 << " rangeYLimit2:" << rangeYLimit2 << endl;				
				rangeYLimit1 = rangeYLimit1 - 0.01;
				rangeYLimit2 = rangeYLimit2 - 0.01;
				if(rangeYLimit1 > 10){j = jmax;}
				j = 0;
			}					
		}
	j++;
	}

	summSideA = abs(summSideA / (float)summSideACounter);
	cout << "nr: " << summSideACounter << " summSideA: " << summSideA << endl;
	summSideB = abs(summSideB / (float)summSideBCounter);
	cout << "nr: " << summSideBCounter  << " summSideB: " << summSideB << endl;	
	summSideC = abs(summSideC / (float)summSideCCounter);
	cout << "nr: " << summSideCCounter << " summSideC: " << summSideC << endl;		

	float deltaZ = summSideA-summSideB;
	cout << "deltaZ: " << deltaZ << endl;
	float deltaZx = summSideB-summSideC;
	cout << "deltaZx: " << deltaZx << endl;	

	float transZ;
	if(summSideA > summSideB){
		transZ = summSideA - (abs(deltaZ)*0.5);
		translationXYZ(*outPcl, *outPcl, 0, 0, transZ);
	}else{
		transZ = summSideB - (abs(deltaZ)*0.5);
		translationXYZ(*outPcl, *outPcl, 0, 0, transZ);
	}

	//calculate rotation for Y axis
	float deltaX = 0.2;
	float hyp = (float)sqrt( (deltaX*deltaX)+(deltaZ*deltaZ) );
	cout << "hyp: " << hyp << endl;
	double alphaPre = deltaZ/hyp;
	float alpha = (float)asin(alphaPre);
	rotateY(*outPcl, *outPcl, -alpha);

	//calculate rotation for X axis
	float hypY = (float)sqrt( (deltaYup*deltaYup)+(deltaZx*deltaZx) );
	cout << "hypY: " << hypY << endl;
	double betaPre = deltaZx/hypY;
	float beta = (float)asin(betaPre);
	rotateX(*outPcl, *outPcl, -beta);	
/*
	if(summSideA > summSideB){
		translationXYZ(*outPcl, *outPcl, 0, 0, -transZ);
	}else{
		translationXYZ(*outPcl, *outPcl, 0, 0, -transZ);
	}
*/

	alpha = (alpha/M_PI)*(float)180.0;
	cout << "alpha: " << alpha << endl;
	beta = (beta/M_PI)*(float)180.0;
	cout << "beta: " << beta << endl;

	out = *outPcl;
}

 /**
  * set wall relative to detected origin XY
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>&
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::createWallPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out){
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr wallPlane (new pcl::PointCloud<pcl::PointXYZRGB>);
	float step = 0.1;
	const float maxOverall = 10;
	float maxHalf = maxOverall * 0.5 * (-1);
	float ix = maxHalf*step; 
	float iy = maxHalf*step; 
	float iz = 0;

	int t=0;
	float tStep=0.5;
	int tmax=10;
	int i=0;
	int j=0;
	int imax=maxOverall;
	int jmax=maxOverall;

	t=0;
	while(t<tmax){
		iy = maxHalf*step; 
		i=0;
		while(i<imax){
			ix = maxHalf*step; 
			j=0;		
			while(j<jmax){
				pcl::PointXYZRGB bufferPoint = pcl::PointXYZRGB(ix, iy, iz, 0,255,0);
				out->push_back(bufferPoint);
				ix = ix + step;
				j++;
			}
			iy = iy + step;	
		i++;
		}
	iz = iz - tStep;
	t++;
	}//end tmax
}

 /**
  * draw axis PCL
  * input:\n
  * \param	int length and float stepSize
  * \return	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  *	x is red(left/right); y is green(up/down), z is blue (depth) \n
  */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclUtilitys::createLineToZaxis(int length, float stepSize){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr axisPcl (new pcl::PointCloud<pcl::PointXYZRGB>);
	float iz = 0; 

	int i=0;
	int imax=length;//100
	while(i<imax){
		pcl::PointXYZRGB axisPointZ = pcl::PointXYZRGB(0,0,-iz ,0,230,230);//Blue
		axisPcl->push_back(axisPointZ);
		iz = iz + stepSize;		
	i++;
	}
	return(axisPcl);
}


 /**
  * draw axis PCL
  * input:\n
  * \param	float x, float y, float z
  * \return	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  *  x is red(left/right); y is green(up/down), z is blue (depth) \n
  */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclUtilitys::createAxis(float x, float y, float z){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr axisPcl (new pcl::PointCloud<pcl::PointXYZRGB>);
	float ix = x; 
	float iy = y; 
	float iz = z; 
	float step = 0.01;

	int i=0;
	int imax=100;
	while(i<imax){
		pcl::PointXYZRGB axisPointX = pcl::PointXYZRGB(ix,y,z ,255,0,0);//Red X axis - left/right
		pcl::PointXYZRGB axisPointY = pcl::PointXYZRGB(x,iy,z ,0,255,0);//Green Y axis - up/down
		pcl::PointXYZRGB axisPointZ = pcl::PointXYZRGB(x,y,iz ,0,0,255);
		axisPcl->push_back(axisPointX);
		ix = ix + step;
		axisPcl->push_back(axisPointY);
		iy = iy + step;
		axisPcl->push_back(axisPointZ);
		iz = iz + step;		
	i++;
	}
	return(axisPcl);
}

 /**
  * draw axis PCL
  * input:\n
  * \param	float x, float y, float z, length and stepSize
  * \return	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  *	x is red(left/right); y is green(up/down), z is blue (depth) \n
  */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclUtilitys::createAxis(float x, float y, float z, int length, float stepSize){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr axisPcl (new pcl::PointCloud<pcl::PointXYZRGB>);
	float ix = x; 
	float iy = y; 
	float iz = z; 
	//float step = 0.01;

	int i=0;
	int imax=length;//100
	while(i<imax){
		pcl::PointXYZRGB axisPointX = pcl::PointXYZRGB(ix,y,z ,255,0,0);//Red X axis - left/right
		pcl::PointXYZRGB axisPointY = pcl::PointXYZRGB(x,iy,z ,0,255,0);//Green Y axis - up/down
		pcl::PointXYZRGB axisPointZ = pcl::PointXYZRGB(x,y,iz ,0,0,255);//Blue
		axisPcl->push_back(axisPointX);
		ix = ix + stepSize;
		axisPcl->push_back(axisPointY);
		iy = iy + stepSize;
		axisPcl->push_back(axisPointZ);
		iz = iz + stepSize;		
	i++;
	}
	return(axisPcl);
}

 /**
  * draw countour from a DINA4 paper
  * input:\n
  * \param	void
  * \return	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclUtilitys::createDINA4PaperContour(){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr axisPcl (new pcl::PointCloud<pcl::PointXYZRGB>);
	float ix = 0; 
	//float iy = 0; 
	float iz = 0; 
	float stepSize = 0.00005;
	float DINA4height = 0.297; //DINA4
	float DINA4width  = 0.210; //DINA4

	int i;
	int imax;

	i=0;
	imax = (int)(DINA4width/stepSize);
	while(i<=imax){
		pcl::PointXYZRGB side01 = pcl::PointXYZRGB(ix,0,0 ,255,255,255);
		axisPcl->push_back(side01);
		pcl::PointXYZRGB side02 = pcl::PointXYZRGB(ix,0,DINA4height ,255,255,255);
		axisPcl->push_back(side02);
		ix = ix + stepSize;	
	i++;
	}

	i=0;
	imax = (int)(DINA4height/stepSize);
	while(i<=imax){
		pcl::PointXYZRGB side03 = pcl::PointXYZRGB(0,0,iz ,255,255,255);
		axisPcl->push_back(side03);
		pcl::PointXYZRGB side04 = pcl::PointXYZRGB(DINA4width,0,iz ,255,255,255);
		axisPcl->push_back(side04);
		iz = iz + stepSize;	
	i++;
	}

	return(axisPcl);
}

 /**
  * load PCL
  * input: path to ply point cloud\n
  * \param	void
  * \return	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclUtilitys::loadPCL(const string pathToPcd){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PCLPointCloud2 readPc2;
	pcl::io::loadPCDFile(pathToPcd, readPc2);
	pcl::fromPCLPointCloud2 (readPc2, *pcl);//Convert
	return(pcl);
}

 /**
  * draw axis PCL
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in float x, float y, float z
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::translationXYZ(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out, float x, float y, float z){
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	transform_1 (0,3) = x;
	transform_1 (1,3) = y;
	transform_1 (2,3) = z;
	pcl::transformPointCloud (in, out, transform_1);
}

 /**
  * invert the z(depth) value
  * input:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& in
  * \return	void
  *
  */
void pclUtilitys::convertZPcl(pcl::PointCloud<pcl::PointXYZRGB>& in){
	int i=0;
	int imax = in.points.size();
	while(i<imax){
		float buffer = in.points[i].z;
		in.points[i].z = (float)(buffer * (-1.0));
	    i++;
	}
}

 /**
  * convert opencv xyz coordinates to PCL viewer xyz coordinates
  * input:\n
  * \param	const double &inx, const double &iny, const double &inz
  * output:\n
  * \param	double &outx, double &outy, double &outz
  * \return	void
  *
  */
void pclUtilitys::convertCordinateSystemFromOpenCvToPlcViewer(const double &inx, const double &iny, const double &inz, double &outx, double &outy, double &outz){
	//mapping to PCL viewer coordinat system --- for worldPos
	/*
	outx = iny; 
	outy = inz; 
	outz = inx*(-1); 
	*/
	outx = iny; //ok
	outy = inz; //ok
	outz = inx; //ok			
	

}

 /**
  * convert opencv xyz coordinates to PCL viewer xyz coordinates
  * input:\n
  * \param	const float &inx, const float &iny, const float &inz
  * output:\n
  * \param	float &outx, float &outy, float &outz
  * \return	void
  *
  */
void pclUtilitys::convertCordinateSystemFromOpenCvToPlcViewer(const float &inx, const float &iny, const float &inz, float &outx, float &outy, float &outz){
	//mapping to PCL viewer coordinat system --- for worldPos
	/*
	outx = iny; 
	outy = inz; 
	outz = inx*(-1); 	
	*/		
	outx = iny; //ok
	outy = inz; //ok
	outz = inx; //ok			
	
}

 /**
  * set color for this PCL
  * input:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& in, float red, float green, float blue
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& in
  * \return	void
  *
  */
void pclUtilitys::setPclRGBColor(pcl::PointCloud<pcl::PointXYZRGB>& in, float r, float g, float b){
	int i=0;
	int imax = in.points.size();
	while(i<imax){
		in.points[i].b = b;
		in.points[i].g = g;
		in.points[i].r = r;
	    i++;
	}
}

 /**
  * convert PCL BGR to RGB
  * input:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& in
  * \return	void
  *
  */
void pclUtilitys::convertXYZBGRtoRGB(pcl::PointCloud<pcl::PointXYZRGB>& in){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr buffer (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(in, *buffer);
	int i=0;
	int imax = in.points.size();
	while(i<imax){
		in.points[i].b = buffer->points[i].r;
		in.points[i].g = buffer->points[i].g;
		in.points[i].r = buffer->points[i].b;
	    i++;
	}
}

 /**
  * convert color PCL to PCL without color
  * input:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::convertXYZRGBtoXYZ(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZ>::Ptr out){
	out->points.resize(in.size());
	int i=0;
	int imax = in.points.size();
	while(i<imax){
	    out->points[i].x = in.points[i].x;
	    out->points[i].y = in.points[i].y;
	    out->points[i].z = in.points[i].z;
	    i++;
	}
}

 /**
  * convert color PCL to PCL without color
  * input:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::convertPtrXYZRGBtoXYZ(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZ>::Ptr& out){
	out->points.resize(in.size());
	int i=0;
	int imax = in.points.size();
	while(i<imax){
	    out->points[i].x = in.points[i].x;
	    out->points[i].y = in.points[i].y;
	    out->points[i].z = in.points[i].z;
	    i++;
	}
}

 /**
  * convert PCL to PCL color (white)
  * input:\n
  * \param	pcl::PointCloud<pcl::PointXYZ>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out
  * \return	void
  *
  */
void pclUtilitys::convertPtrXYZtoXYZRGB(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out){
	out->points.resize(in.size());
	int i=0;
	int imax = in.points.size();
	while(i<imax){
	    out->points[i].x = in.points[i].x;
	    out->points[i].y = in.points[i].y;
	    out->points[i].z = in.points[i].z;
		out->points[i].r = 255;
		out->points[i].g = 255;
		out->points[i].b = 255;
	    i++;
	}
}

 /**
  * convert color PCL to PCL without color with normal
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZINormal>::Ptr out
  * \return	void
  *
  */
void pclUtilitys::convertXYZRGBtoXYZI(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZINormal>::Ptr out){
	pcl::PointCloud<pcl::Normal>::Ptr normalAtWork (new pcl::PointCloud<pcl::Normal>);
	getNormalsFromCloudRgb(in, *normalAtWork);

	out->points.resize(in.size());
	int i=0;
	int imax = in.points.size();
	while(i<imax){
	    out->points[i].x = in.points[i].x;
	    out->points[i].y = in.points[i].y;
	    out->points[i].z = in.points[i].z;
		out->points[i].normal_x = normalAtWork->points[i].normal_x;
		out->points[i].normal_y = normalAtWork->points[i].normal_y;
		out->points[i].normal_z = normalAtWork->points[i].normal_z;		
	    i++;
	}
}

 /**
  * convert color PCL to PCL color with normal
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZINormal>::Ptr out
  * \return	void
  *
  */
void pclUtilitys::convertXYZRGBtoXYZRGBNormal(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr out){
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRgbIn (new pcl::PointCloud<pcl::PointXYZ>);
	//*cloudRgbIn = in;
	pcl::PointCloud<pcl::Normal>::Ptr normalAtWork (new pcl::PointCloud<pcl::Normal>);
	getNormalsFromCloudRgb(in, *normalAtWork);

	out->points.resize(in.size());
	int i=0;
	int imax = in.points.size();
	while(i<imax){
	    out->points[i].x = in.points[i].x;
	    out->points[i].y = in.points[i].y;
	    out->points[i].z = in.points[i].z;
	    out->points[i].r = in.points[i].r;
	    out->points[i].g = in.points[i].g;
	    out->points[i].b = in.points[i].b;
		out->points[i].normal_x = normalAtWork->points[i].normal_x;
		out->points[i].normal_y = normalAtWork->points[i].normal_y;
		out->points[i].normal_z = normalAtWork->points[i].normal_z;
	    i++;
	}
}

/**
  * convert color PCL with normal to PCL color
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGBNormal>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out
  * \return	void
  *
  */
void pclUtilitys::convertXYZRGBNormaltoXYZRGB(const pcl::PointCloud<pcl::PointXYZRGBNormal>& in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out){
	out->points.resize(in.size());
	int i=0;
	int imax = in.points.size();
	while(i<imax){
	    out->points[i].x = in.points[i].x;
	    out->points[i].y = in.points[i].y;
	    out->points[i].z = in.points[i].z;
	    out->points[i].r = in.points[i].r;
	    out->points[i].g = in.points[i].g;
	    out->points[i].b = in.points[i].b;	
	    i++;
	}
}


/**
  * convert PCL no color with normal to PCL color
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZINormal>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out
  * \return	void
  *
  */
void pclUtilitys::convertXYZItoXYZRGB(const pcl::PointCloud<pcl::PointXYZINormal>& in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out){
	out->points.resize(in.size());
	int i=0;
	int imax = in.points.size();
	while(i<imax){
	    out->points[i].x = in.points[i].x;
	    out->points[i].y = in.points[i].y;
	    out->points[i].z = in.points[i].z;
	    i++;
	}
}

/**
  * convert PCL no color with normal to PCL color
  * input:\n
  * \param	const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b
  * output:\n
  * \param	float squared_distance
  * \return	void
  *
  */
bool pclUtilitys::customRegionGrowing(const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b, float squared_distance)
{
  Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), point_b_normal = point_b.getNormalVector3fMap ();
  if (squared_distance < 10000)
  {
    if (std::abs (point_a.intensity - point_b.intensity) < 8.0f)
      return (true);
    if (std::abs (point_a_normal.dot (point_b_normal)) < 0.06)
      return (true);
  }
  else
  {
    if (std::abs (point_a.intensity - point_b.intensity) < 3.0f)
      return (true);
  }
  return (false);
}


//does not work in a propper way!!!!
void pclUtilitys::extractPlaneFromCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& cloudOut, bool bSegPlane){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	*cloudIn = cloud;
	cout << "1 start: " << cloudIn->size() << endl;

	//SAC plane segmentation
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (true);// Optional

	cout << "2 start: " << cloudIn->size() << endl;

	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);// pcl::SACMODEL_NORMAL_PLANE //SACMODEL_PLANE // SACMODEL_PERPENDICULAR_PLANE
	seg.setMethodType (pcl::SAC_RANSAC); // pcl::SAC_RANSAC
	seg.setDistanceThreshold (0.20); //////////////////////////// 0.20
	seg.setInputCloud(cloudIn);
	cout << " ------ " << endl;
	seg.segment (*inliers, *coefficients);
	if (inliers->indices.size () == 0){
		cout << "Could not estimate a planar model for the given dataset" << endl;
	}else{
		cout << "Inliers: " << inliers->indices.size() << endl;
	}



/*
    // Extract the inliers - no rgb
	pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloudIn);
    extract.setIndices (inliers);
    extract.setNegative (bSegPlane);
    cout << " start: " << cloudIn->size() << endl;
    extract.filter(*cloudOutTemp);
    cout << " End: " << cloudOutTemp->size() << endl;
    cloudOut = *cloudOutTemp;
    */
}

/**
  * get PCL plane from color PCL
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& cloud, bool bSegPlane
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& cloudOut
  * \return	void
  *
  */
void pclUtilitys::extractPlaneFromCloudRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut, bool bSegPlane){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = cloud;
	//SAC plane segmentation
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;// Create the segmentation object
	seg.setOptimizeCoefficients (true);// Optional
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);// pcl::SACMODEL_NORMAL_PLANE //
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.10); //////////////////////////// 0.20
	seg.setInputCloud(cloudIn);
	seg.segment (*inliers, *coefficients);
    // Extract the inliers - no rgb
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloudIn);
    extract.setIndices (inliers);
    extract.setNegative (bSegPlane);
    extract.filter(cloudOut);
}

/**
  * momentOfInertiaTest
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZ>& out
  * \return	void
  *
  */
void pclUtilitys::momentOfInertiaTest(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointXYZ>& out){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	*cloudIn = in;

	  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	  feature_extractor.setInputCloud (cloudIn);
	  feature_extractor.compute ();

	  std::vector <float> moment_of_inertia;
	  std::vector <float> eccentricity;
	  pcl::PointXYZ min_point_AABB;
	  pcl::PointXYZ max_point_AABB;
	  pcl::PointXYZ min_point_OBB;
	  pcl::PointXYZ max_point_OBB;
	  pcl::PointXYZ position_OBB;
	  Eigen::Matrix3f rotational_matrix_OBB;
	  float major_value, middle_value, minor_value;
	  Eigen::Vector3f major_vector, middle_vector, minor_vector;
	  Eigen::Vector3f mass_center;

	  feature_extractor.getMomentOfInertia (moment_of_inertia);
	  feature_extractor.getEccentricity (eccentricity);
	  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
	  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
	  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
	  feature_extractor.getMassCenter (mass_center);

	  ////////////////////////////////////////////////////////
	  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	  viewer->setBackgroundColor (0, 0, 0);
	  viewer->addCoordinateSystem (1.0);
	  viewer->initCameraParameters ();
	  viewer->addPointCloud<pcl::PointXYZ> (cloudIn, "sample cloud");
	  viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
	  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

	  Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
	  Eigen::Quaternionf quat (rotational_matrix_OBB);
	  viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
	  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

	  pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
	  pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
	  pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
	  pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
	  viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
	  viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
	  viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

	  while(!viewer->wasStopped())
	  {
	    viewer->spinOnce (100);
	    std::this_thread::sleep_for(100ms);
	  }
}

/**
  * momentOfInertiaTestRGB
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::momentOfInertiaTestRgb(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRgbIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudRgbIn = in;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	convertXYZRGBtoXYZ(*cloudRgbIn, cloudIn);

	  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	  feature_extractor.setInputCloud (cloudIn);
	  feature_extractor.compute ();

	  std::vector <float> moment_of_inertia;
	  std::vector <float> eccentricity;
	  pcl::PointXYZ min_point_AABB;
	  pcl::PointXYZ max_point_AABB;
	  pcl::PointXYZ min_point_OBB;
	  pcl::PointXYZ max_point_OBB;
	  pcl::PointXYZ position_OBB;
	  Eigen::Matrix3f rotational_matrix_OBB;
	  float major_value, middle_value, minor_value;
	  Eigen::Vector3f major_vector, middle_vector, minor_vector;
	  Eigen::Vector3f mass_center;

	  feature_extractor.getMomentOfInertia (moment_of_inertia);
	  feature_extractor.getEccentricity (eccentricity);
	  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
	  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
	  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
	  feature_extractor.getMassCenter (mass_center);

	  ////////////////////////////////////////////////////////
	  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	  viewer->setBackgroundColor (0, 0, 0);
	  viewer->addCoordinateSystem (1.0);
	  viewer->initCameraParameters ();
	  viewer->addPointCloud<pcl::PointXYZRGB> (cloudRgbIn, "sample cloud");
	  viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
	  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

	  Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
	  Eigen::Quaternionf quat (rotational_matrix_OBB);
	  viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
	  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

	  pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
	  pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
	  pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
	  pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
	  viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
	  viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
	  viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

	  while(!viewer->wasStopped())
	  {
	    viewer->spinOnce (100);
	    std::this_thread::sleep_for(100ms);
	  }
}

/**
  * voxel grid
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& cloud, float distanceThreshold
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZ>& cloudOut
  * \return	void
  *
  */
void pclUtilitys::voxelGrid(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& cloudOut, float distanceThreshold){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	*cloudIn = cloud;
	//conv pclXYZ to pcl2
	//pcl::PCLPointCloud2::Ptr cloudInPcl2 (new pcl::PCLPointCloud2 ());
	//pcl::toPCLPointCloud2(*cloudIn, *cloudInPcl2);

	///////////////////////////////////////////
	//Voxel grid downsample
	//pcl::PCLPointCloud2::Ptr cloudPcl2filtered (new pcl::PCLPointCloud2 ());

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloudIn);
	sor.setLeafSize (distanceThreshold, distanceThreshold, distanceThreshold);//0.04f
	//sor.filter (*cloudPcl2filtered);
	sor.filter (cloudOut);

	///////////////////////////////////////////
	//conv pcl2 to pclXYZ
	//pcl::fromPCLPointCloud2(*cloudPcl2filtered, cloudOut);
	//test: pcl::fromPCLPointCloud2(*cloudInPcl2, cloudOut);
}

/**
  * voxel grid color
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& cloud, float distanceThreshold
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& cloudOut
  * \return	void
  *
  */
void pclUtilitys::voxelGridRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut, float distanceThreshold){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = cloud;
	//conv pclXYZ to pcl2
	//pcl::PCLPointCloud2::Ptr cloudInPcl2 (new pcl::PCLPointCloud2 ());
	//pcl::toPCLPointCloud2(*cloudIn, *cloudInPcl2);
	///////////////////////////////////////////
	//Voxel grid downsample
	pcl::PCLPointCloud2::Ptr cloudPcl2filtered (new pcl::PCLPointCloud2 ());
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloudIn);
	//float distanceThreshold = 0.04f;
	sor.setLeafSize (distanceThreshold, distanceThreshold, distanceThreshold);
	sor.filter (cloudOut);
	//conv pcl2 to pclXYZ
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRgbFiltered (new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::fromPCLPointCloud2(*cloudPcl2filtered, cloudOut);
}

/**
  * cliping z (depth) based
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& cloud, float distanceThreshold, float minfl, float maxfl
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZ>& cloudOut
  * \return	void
  *
  */
//Cliping: schneidet hier nur den Bereich in Z Richtung ab.
void pclUtilitys::passTroughFilter(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& cloudOut, float minfl, float maxfl){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	*cloudIn = cloud;
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloudIn);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (minfl, maxfl);
	//pass.setNegative (true);
	pass.filter(cloudOut);
}

/**
  * cliping z (depth) based
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& cloud, float distanceThreshold, float minfl, float maxfl
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& cloudOut
  * \return	void
  *
  */
//Cliping: schneidet hier nur den Bereich in Z Richtung ab.
void pclUtilitys::passTroughFilterRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut, float minfl, float maxfl){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = cloud;
	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(cloudIn);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (minfl, maxfl);
	//pass.setNegative (true);
	pass.filter(cloudOut);
}

/**
  * cliping x and y 
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& cloud, float distanceThreshold, float minX, float maxX , float minY, float maxY
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& cloudOut
  * \return	void
  *
  */
void pclUtilitys::cropFilterRgb(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut, float minX, float maxX, float minY, float maxY){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = in;
	cloudOut.clear();

	int i = 0;
	int imax = cloudIn->size();
	while(i<imax){
		if( (cloudIn->points[i].x > minX)&&(cloudIn->points[i].x < maxX)&&(cloudIn->points[i].y > minY)&&(cloudIn->points[i].y < maxY) ){
			cloudOut.push_back(cloudIn->points[i]);
		}
	i++;
	}	
}

/**
  * statisticalOutlierRemovalFilter
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& cloud, float distanceThreshold, int numbersOfNeighbors, float deviationMul
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZ>& cloudOut
  * \return	void
  *
  */
void pclUtilitys::statisticalOutlierRemovalFilter(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& cloudOut, int numbersOfNeighbors, float deviationMul){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	*cloudIn = cloud;
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloudIn);
	sor.setMeanK(numbersOfNeighbors);//50
	sor.setStddevMulThresh(deviationMul);//1.0
	sor.filter(cloudOut);
}

/**
  * movingLeastSquaresFilerRgb
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& cloud, float distanceThreshold, int polyOrder, float radius
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& cloudOut
  * \return	void
  *
  */
//example: https://github.com/PointCloudLibrary/pcl/blob/master/test/surface/test_moving_least_squares.cpp
void pclUtilitys::movingLeastSquaresFilerRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut, int polyOrder, float radius){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = cloud;

	//pcl::PointCloud<PointXYZ> mls_points;
	//pcl::PointCloud<PointNormal>::Ptr mls_normals (new PointCloud<PointNormal> ());
	pcl::MovingLeastSquares<PointXYZRGB, PointNormal> mls;


	/*
  mls.setInputCloud (cloud);
  mls.setComputeNormals (true);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);
	*/
}

/**
  * statisticalOutlierRemovalFilterRgb
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& cloud, float distanceThreshold, int numbersOfNeighbors, float deviationMul
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& cloudOut
  * \return	void
  *
  */
void pclUtilitys::statisticalOutlierRemovalFilterRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut, int numbersOfNeighbors, float deviationMul){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = cloud;
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloudIn);
	sor.setMeanK(numbersOfNeighbors);//50
	sor.setStddevMulThresh(deviationMul);//1.0
	sor.filter(cloudOut);
}

/**
  * normal from cloud
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& cloud
  * output:\n
  * \param	pcl::PointCloud<pcl::PointNormal>& normals
  * \return	void
  *
  */
void pclUtilitys::getNormalsFromCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointNormal>& normals){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	*cloudIn = cloud;
	const float radius = 0.02;//0.05
	const float k = 0; //must be 0

	if (cloudIn->isOrganized ()){
	  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
	  ne.setInputCloud (cloudIn);
	  ne.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::PointNormal>::COVARIANCE_MATRIX);
	  ne.setNormalSmoothingSize (radius);
	  ne.setDepthDependentSmoothing (true);
	  ne.compute (normals);
	}else{
	    // Estimate the normals
        pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr normalKdTree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setInputCloud(cloudIn);
        ne.setSearchMethod(normalKdTree);
		ne.setKSearch(k);
        ne.setRadiusSearch(radius); //0.02
        ne.compute(normals);
	}
}

/**
  * normals from RGB cloud
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& cloud
  * output:\n
  * \param	pcl::PointCloud<pcl::Normal>& normals
  * \return	void
  *
  */
void pclUtilitys::getNormalsFromCloudRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::Normal>& normals){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudInRGB = cloud;
	const float radius = 0.05;
	const float k = 0; //must be 0

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	convertXYZRGBtoXYZ(*cloudInRGB, cloudIn);

	if (cloudIn->isOrganized ()){
	  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	  ne.setInputCloud (cloudIn);
	  ne.setNormalEstimationMethod (pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::COVARIANCE_MATRIX);
	  ne.setNormalSmoothingSize (float (radius));
	  ne.setDepthDependentSmoothing (true);
	  ne.compute (normals);
	}else{
	  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	  ne.setInputCloud (cloudIn);
	  //ne.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
	  ne.setSearchMethod (pclUtilitys::tree_xyz);
	  ne.setKSearch(k);
	  ne.setRadiusSearch (radius);
	  ne.compute (normals);
	}
}

/**
  * point normals from RGB cloud
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& cloud
  * output:\n
  * \param	pcl::PointCloud<pcl::PointNormal>& normals
  * \return	void
  *
  */
void pclUtilitys::getPointNormalsFromCloudRgb(const pcl::PointCloud<pcl::PointXYZRGB>& input, pcl::PointCloud<pcl::PointNormal>& cloud_with_normals){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pclUtilitys::convertXYZRGBtoXYZ(input, cloud);

	// Normal estimation
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	// Concatenate the XYZ and normal fields
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, cloud_with_normals);	
}

/**
  * get smooth point normals from RGB cloud
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& cloud
  * output:\n
  * \param	pcl::PointCloud<pcl::PointNormal>& smooth normals
  * \return	void
  *
  */
void pclUtilitys::getSmoothPointNormalsFromCloudRgb(const pcl::PointCloud<pcl::PointXYZRGB>& input, pcl::PointCloud<pcl::PointNormal>& mls_points){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pclUtilitys::convertXYZRGBtoXYZ(input, cloud);

	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals (true);

	// Set parameters
	mls.setInputCloud (cloud);
	mls.setPolynomialOrder (2);//2
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.02);//0.03

	// Reconstruct
	mls.process (mls_points);
}

/**
  * normal from RGB cloud to RGBI
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& cloud
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZINormal>& cloudOut
  * \return	void
  *
  */
void pclUtilitys::getNormalsXYZIFromCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZINormal>& cloudOut){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	*cloudIn = cloud;
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointXYZINormal> ne;
	ne.setInputCloud (cloudIn);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());	
	//ne.setSearchMethod (tree);
	ne.setSearchMethod (pclUtilitys::tree_xyz);
	// Output datasets
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
	// Use all neighbors in a sphere of radius 5cm
	ne.setRadiusSearch (0.05);
	// Compute the features
	ne.compute (cloudOut);
}

/**
  * get NormalsXYZI from Cloud RGB
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& cloud
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZINormal>& cloudOut
  * \return	void
  *
  */
void pclUtilitys::getNormalsXYZIFromCloudRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZINormal>& cloudOut){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = cloud;
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZINormal> ne;
	ne.setInputCloud (cloudIn);
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	//ne.setSearchMethod (tree);
	ne.setSearchMethod (pclUtilitys::tree_xyzrgb);
	// Output datasets
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
	// Use all neighbors in a sphere of radius 5cm
	ne.setRadiusSearch (0.05);
	// Compute the features
	ne.compute (cloudOut);
}

/**
  * doConditionalEuclideanClusteringRgb
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& cloud
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& cloudOut
  * \return	void
  *
  */
void pclUtilitys::doConditionalEuclideanClusteringRgb(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFinalOut (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = cloud;
	pcl::copyPointCloud(*cloudIn, *cloudFinalOut);
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudNormals (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);

	pcl::PointCloud<pcl::PointXYZINormal>::Ptr clout (new pcl::PointCloud<pcl::PointXYZINormal>);

	convertXYZRGBtoXYZI(*cloudIn, clout);//copy in to out --- from PointXYZRGB to PointXYZI
	//pcl::copyPointCloud(*cloudIn, *clout);//copy in to out --- from PointXYZRGB to PointXYZI

	getNormalsXYZIFromCloudRgb(*cloudIn, *cloudNormals);

	std::vector<pcl::PointIndices> cluster_indices;
	 pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> cec (true);
	  cec.setInputCloud(cloudNormals);
	  //cec.setConditionFunction(&pclUtilitys::customRegionGrowing);
	  cec.setClusterTolerance (0.02);
	  cec.setMinClusterSize (cloudNormals->size () / 1000);
	  cec.setMaxClusterSize (cloudNormals->size () / 5);
	  //cec.setMinClusterSize (60);
	  //cec.setMaxClusterSize (8000);
	  cec.segment (*clusters);
	  cec.getRemovedClusters (small_clusters, large_clusters);

	  cout << "doConditionalEuclideanClusteringRgb ... "<< endl;
	  cout << "clusters size: " << clusters->size() << endl;
	  cout << "small_clusters size: " << small_clusters->size() << endl;
	  cout << "large_clusters size: " << large_clusters->size() << endl;

	  //////////////////
	  /*
	  // Using the intensity channel for lazy visualization of the output
	  for (int i = 0; i < small_clusters->size (); ++i)
	    for (int j = 0; j < (*small_clusters)[i].indices.size (); ++j)
	      (*clout)[(*small_clusters)[i].indices[j]].intensity = -2.0;
	  for (int i = 0; i < large_clusters->size (); ++i)
	    for (int j = 0; j < (*large_clusters)[i].indices.size (); ++j)
	      (*clout)[(*large_clusters)[i].indices[j]].intensity = +10.0;
	  for (int i = 0; i < clusters->size (); ++i)
	  {
	    int label = rand () % 8;
	    for (int j = 0; j < (*clusters)[i].indices.size (); ++j)
	      (*clout)[(*clusters)[i].indices[j]].intensity = label;
	  }
	  */
	 // convertXYZItoXYZRGB(*clout, cloudFinalOut);//copy in to out --- from PointXYZRGB to PointXYZI
	 // cloudOut = *cloudFinalOut;//rgb

}

/**
  * create Range Image from Pcl
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& cloud
  * output:\n
  * \param	pcl::RangeImage& rangeImage
  * \return	void
  *
  */
void pclUtilitys::createRangeImageFromPcl(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::RangeImage& rangeImage){
	 pcl::PointCloud<pcl::PointXYZ> pointCloud;
	 pointCloud = cloud;
	  //cout << "createRangeImageFromPcl " << endl;
	  //cout << "Generate the data " << endl;
	  /*
	  // Generate the data
	  for (float y=-0.5f; y<=0.5f; y+=0.01f) {
	    for (float z=-0.5f; z<=0.5f; z+=0.01f) {
	      pcl::PointXYZ point;
	      point.x = 2.0f - y;
	      point.y = y;
	      point.z = z;
	      pointCloud.push_back(point);
	    }
	  }

	  pointCloud.width = pointCloud.size();
	  pointCloud.height = 1;
	  */
	  //cout << "init " << endl;
	  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
	  float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
	  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
	  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
	  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	  float noiseLevel=0.00;
	  float minRange = 0.0f;
	  int borderSize = 1;
	  //cout << "createFromPointCloud ... " << endl;
	  ///pcl::RangeImage rangeImage;
	  rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
	                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	  //cout << "Range image ... " << endl;
	 // std::cout << rangeImage << "\n";
}

/**
  * radiusOutlierRemovalhelpFilter
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& in, float radius, int numbersOfNeighbors
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZ>& out
  * \return	void
  *
  */
void pclUtilitys::radiusOutlierRemovalhelpFilter(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointXYZ>& out, int mode, float radius, int numbersOfNeighbors){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	*cloudIn = in;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutTemp (new pcl::PointCloud<pcl::PointXYZ>);
	//*cloudIn = cloud;
	//pcl::copyPointCloud(cloud, *cloudIn);

	//int mode = 1; int numbersOfNeighbors = 2; float radius = 0.8;
	if (mode == 0){
	  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	  // build the filter
	  outrem.setInputCloud(cloudIn);
	  outrem.setRadiusSearch(radius); //0.8
	  outrem.setMinNeighborsInRadius (numbersOfNeighbors);  //2
	  outrem.setKeepOrganized(true);
	  // apply filter
	  outrem.filter(out);
	  //pcl::copyPointCloud(*cloudOutTemp, out);
	}
	else if (mode == 1){
	  // build the condition
	  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
		pcl::ConditionAnd<pcl::PointXYZ> ());
	  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
		pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
	  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
		pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
	  // build the filter
	  pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
	  condrem.setCondition (range_cond);
	  condrem.setInputCloud (cloudIn);
	  condrem.setKeepOrganized(true);
	  // apply filter
	  condrem.filter(out);
	  //pcl::copyPointCloud(*cloudOutTemp, cloudOut);
	}
}

/**
  * radiusOutlierRemovalhelpFilter
  * input:\n
  * \param	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud
  * \return	void 
  *
  */
void pclUtilitys::ViewSimplePcl (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
}


/**
  * radiusOutlierRemovalhelpFilter
  * input:\n
  * \param	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud
  * \return	pcl::visualization::PCLVisualizer::Ptr
  *
  */
 /*
pcl::visualization::PCLVisualizer::Ptr pclUtilitys::ViewSimplePcl (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
*/


/**
  * visualisation from RGB cloud
  * input:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud
  * \return	void  ///pcl::visualization::PCLVisualizer::Ptr
  *
  */
//pcl::visualization::PCLVisualizer::Ptr pclUtilitys::viewRgb (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
void pclUtilitys::viewRgb(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  //return (viewer);
}

/**
  * visualisation from two RGB clouds
  * input:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud2
  * \return	pcl::visualization::PCLVisualizer::Ptr
  *
  */
pcl::visualization::PCLVisualizer::Ptr pclUtilitys::viewRgbDouble (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud2)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud2, rgb, "sample cloud2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

/**
  * visualisation normals from cloud
  * input:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals
  * \return	pcl::visualization::PCLVisualizer::Ptr
  *
  */
pcl::visualization::PCLVisualizer::Ptr pclUtilitys::viewRgbNormals (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

/**
  * visualisation via viewports cloud without and RGB cloud
  * input:\n
  * \param	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRgb
  * \return	pcl::visualization::PCLVisualizer::Ptr
  *
  */
pcl::visualization::PCLVisualizer::Ptr pclUtilitys::viewportsVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRgb)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudRgb);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloudRgb, rgb, "sample cloud2", v2);

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
  viewer->addCoordinateSystem (1.0);

  return (viewer);
}

/**
  * visualisation via viewports two RGB cloud
  * input:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRgb1, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRgb2
  * \return	pcl::visualization::PCLVisualizer::Ptr
  *
  */
pcl::visualization::PCLVisualizer::Ptr pclUtilitys::viewportsDiffVisRgb (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRgb1, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudRgb2){
	  rootPclRgbSize = (int)cloudRgb1->size();
	  rootPclRgbResultSize = (int)cloudRgb2->size();

	  // --------------------------------------------------------
	  // ----- Show the differenze between tow PCL          -----
	  // --------------------------------------------------------
	  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	  viewer->initCameraParameters ();
	  int v1(0);
	  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	  viewer->setBackgroundColor (0, 0, 0, v1);
	  //viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
	  std::string strTextv1 = std::to_string(rootPclRgbSize) + std::string(" Points");
	  viewer->addText(strTextv1, 10, 10, "v1 text", v1);
	  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudRgb1);
	  viewer->addPointCloud<pcl::PointXYZRGB> (cloudRgb1, rgb, "sample cloud1", v1);

	  int v2(0);
	  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	  viewer->setBackgroundColor (0.9, 0.9, 0.9, v2);
	  //viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	  std::string strTextv2 = std::to_string(rootPclRgbResultSize) + std::string(" Points");
	  viewer->addText(strTextv2, 10, 10, "v2 text", v2);
	  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloudRgb2);
	  viewer->addPointCloud<pcl::PointXYZRGB> (cloudRgb2, rgb2, "sample cloud2", v2);

	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
	  viewer->addCoordinateSystem (1.0);

	  return (viewer);
}

/**
  * visualisation via viewports two cloud without color
  * input:\n
  * \param	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2
  * \return	pcl::visualization::PCLVisualizer::Ptr
  *
  */
pcl::visualization::PCLVisualizer::Ptr pclUtilitys::viewportsDiffVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2){
	  rootPclSize = (int)cloud1->size();
	  rootPclResultSize = (int)cloud2->size();

	  // --------------------------------------------------------
	  // ----- Show the differenze between tow PCL          -----
	  // --------------------------------------------------------
	  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	  viewer->initCameraParameters ();
	  int v1(0);
	  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	  viewer->setBackgroundColor (0, 0, 0, v1);
	  std::string strTextv1 = std::to_string(rootPclSize) + std::string(" Points");
	  viewer->addText(strTextv1, 10, 10, "v1 text", v1);
	  viewer->addPointCloud<pcl::PointXYZ> (cloud1, "sample cloud1", v1);

	  int v2(0);
	  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
	  std::string strTextv2 = std::to_string(rootPclResultSize) + std::string(" Points");
	  viewer->addText(strTextv2, 10, 10, "v2 text", v2);
	  viewer->addPointCloud<pcl::PointXYZ> (cloud2, "sample cloud2", v2);

	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud1");
	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
	  viewer->addCoordinateSystem (1.0);

	  //viewer->initCameraParameters ();
	  return (viewer);
}

/*
pcl::visualization::PCLVisualizer::Ptr simpleXZIVis (pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZI> rgbi(cloud);
  viewer->addPointCloud<pcl::PointXYZI> (cloud, rgbi, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
*/

/**
  * set viewer pose
  * input:\n
  * \param	pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose
  * \return	void
  *
  */
void pclUtilitys::setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);

}
/*
pcl::visualization::PCLVisualizer::Ptr pclUtilitys::simpleRangeImgVis(pcl::RangeImage::Ptr& range_img_ptr){

	pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
	range_image_ptr = range_img_ptr;

    //pcl::RangeImage& range_image = *range_image_ptr;
	*range_image = *range_image_ptr;

	pcl::visualization::PCLVisualizer::Ptr viewerOut (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	pcl::visualization::PCLVisualizer viewer ("3D img Viewer");
	viewer.setBackgroundColor (1, 1, 1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
	viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
	//viewer.addCoordinateSystem (1.0f, "global");
	//PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
	//viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
	viewer.initCameraParameters ();
	setViewerPose(viewer, range_image->getTransformationToWorldSystem ());

	// --------------------------
	// -----Show range image-----
	// --------------------------
	//visualization::RangeImageVisualizer range_image_widget ("Range image");
	range_image_widget.showRangeImage (*range_image);
	*viewerOut = viewer;
	return (viewerOut);
}
*/

/**
  * do euclidean clustering
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZ>& out
  * \return	void
  *
  */
void pclUtilitys::doEuclideanClustering(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointXYZ>& out){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	*cloudIn = in;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloudIn);

	//extracting
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.02);// 0.02 = 2cm
	ec.setMinClusterSize (60);
	ec.setMaxClusterSize (400);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloudIn);
	ec.extract (cluster_indices);
	//cout << "found clusters: " << cluster_indices.size() << endl;

	int comidx = 0;
	int idx=0;
	int idxmax=0;
	int i=0;
	int imax=cluster_indices.size();
	while(i<imax){
		idx=0;
		idxmax = cluster_indices[i].indices.size();
		//cout << "  clusters nr: " << i << " size: " << idxmax << endl;
		while(idx<idxmax){
			out.push_back((*cloudIn)[comidx]);
		idx++;
		comidx++;
		}
	i++;
	}
}

/**
  * do euclidean clustering with RGB cloud
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::doEuclideanClusteringRgb(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = in;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (cloudIn);

	//extracting
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance (0.02); // 0.02 = 2cm
	ec.setMinClusterSize (60);
	ec.setMaxClusterSize (600);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloudIn);
	ec.extract (cluster_indices);
	cout << "found clusters: " << cluster_indices.size() << endl;

	int comidx = 0;
	int idx=0;
	int idxmax=0;
	int i=0;
	int imax=cluster_indices.size();
	while(i<imax){
		idx=0;
		idxmax = cluster_indices[i].indices.size();
		cout << "  clusters nr: " << i << " size: " << idxmax << endl;
		while(idx<idxmax){
			out.push_back((*cloudIn)[comidx]);
		idx++;
		comidx++;
		}
	i++;
	}
}

/**
  * test do euclidean clustering with RGB cloud
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::testDoEuclideanClusteringRgb(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = in;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud (cloudIn);

	//extracting
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance (0.01); // 0.02 = 2cm
	ec.setMinClusterSize (60);
	ec.setMaxClusterSize (4000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloudIn);
	ec.extract (cluster_indices);
	cout << "found clusters: " << cluster_indices.size() << endl;

	int comidx = 0;
	int idx=0;
	int idxmax=0;
	int i=0;
	int imax=cluster_indices.size();
	while(i<imax){
		idx=0;
		idxmax = cluster_indices[i].indices.size();
		cout << "  clusters nr: " << i << " size: " << idxmax << endl;
		while(idx<idxmax){
			if(idxmax > 120){
				out.push_back((*cloudIn)[comidx]);
			}
		idx++;
		comidx++;
		}
	i++;
	}
}

/**
  * doRegionGrowingSegmentationColor with RGB cloud
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::doRegionGrowingSegmentationColor(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = in;
	pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::removeNaNFromPointCloud (*cloudIn, *indices);

	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud (cloudIn);
	reg.setIndices (indices);
	reg.setSearchMethod (tree);
	reg.setDistanceThreshold (0.08);
	reg.setPointColorThreshold (30);
	reg.setRegionColorThreshold (5);
	reg.setMinClusterSize (200);

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);

	cout << "doRegionGrowingSegmentation cluster size: " << clusters.size() << endl;

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
	out = *colored_cloud;
}

/**
  * doRegionGrowingSegmentationColor with cloud without color input
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::doRegionGrowingSegmentation(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointXYZRGB>& out){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	*cloudIn = in;

	pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (cloudIn);
	normal_estimator.setKSearch (50);
	normal_estimator.compute (*normals);

	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::removeNaNFromPointCloud(*cloudIn, *indices);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize (50);
	reg.setMaxClusterSize (1000000);
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (30);
	reg.setInputCloud (cloudIn);
	reg.setIndices (indices);
	reg.setInputNormals (normals);
	reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold (1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
	out = *colored_cloud;
}


/**
  * momentOfInertiaRgb
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::momentOfInertiaRgb(const pcl::PointCloud<pcl::PointXYZRGB>& in){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRgbIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudRgbIn = in;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	convertXYZRGBtoXYZ(*cloudRgbIn, cloudIn);

	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud (cloudIn);
	feature_extractor.compute ();

	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;
	pcl::PointXYZ min_point_AABB;
	pcl::PointXYZ max_point_AABB;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;

	feature_extractor.getMomentOfInertia (moment_of_inertia);
	feature_extractor.getEccentricity (eccentricity);
	feature_extractor.getAABB (min_point_AABB, max_point_AABB);
	feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues (major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter (mass_center);

	////////////////////////////////////////////////////////
	//viewer->addPointCloud<pcl::PointXYZRGB> (cloudRgbIn, "sample cloud");
	//axis aligned AABB
	//viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

	//obj aligened OBB
	Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
	Eigen::Quaternionf quat (rotational_matrix_OBB);
	viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
	/*
	pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
	pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
	pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
	pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
	viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
	viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
	viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
	*/
}

/**
  * test random sample consensus
  * \return	void
  *
  */
void pclUtilitys::testRandomSampleConsensus(){
	  // initialize PointClouds
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

	  // populate our PointCloud with points
	  cloud->width    = 500;
	  cloud->height   = 1;
	  cloud->is_dense = false;
	  cloud->points.resize (cloud->width * cloud->height);
	  long int maxP = cloud->size();

	  //for (pcl::index_t i = 0; i < static_cast<pcl::index_t>(cloud->size ()); ++i)
	  for (long int i = 0; i < maxP; ++i)
	  {
		  /**
	    if (pcl::console::find_argument (argc, argv, "-s") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
	    {
	      (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0);
	      (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0);
	      if (i % 5 == 0)
	        (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0);
	      else if(i % 2 == 0)
	        (*cloud)[i].z =  sqrt( 1 - ((*cloud)[i].x * (*cloud)[i].x)
	                                      - ((*cloud)[i].y * (*cloud)[i].y));
	      else
	        (*cloud)[i].z =  - sqrt( 1 - ((*cloud)[i].x * (*cloud)[i].x)
	                                        - ((*cloud)[i].y * (*cloud)[i].y));
	    }
	    else
	    {
	    	*/
	      (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0);
	      (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0);
	      if( i % 2 == 0)
	        (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0);
	      else
	        (*cloud)[i].z = -1 * ((*cloud)[i].x + (*cloud)[i].y);
	    }

	 // }

	  std::vector<int> inliers;

	  // created RandomSampleConsensus object and compute the appropriated model
	  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
	    model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));
	  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
	    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

	    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	    ransac.setDistanceThreshold (.01);
	    ransac.computeModel();
	    ransac.getInliers(inliers);


	  // copies all inliers of the model computed to another PointCloud
	  pcl::copyPointCloud (*cloud, inliers, *final);

	  // creates the visualization object and adds either our original cloud or all of the inliers
	  // depending on the command line arguments specified.
	 // pcl::visualization::PCLVisualizer::Ptr viewer;
	  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	  viewer->setBackgroundColor (0, 0, 0);
	  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	  //viewer->addCoordinateSystem (1.0, "global");
	  viewer->initCameraParameters ();
	  while (!viewer->wasStopped ())
	  {
	    viewer->spinOnce (100);
	    std::this_thread::sleep_for(100ms);
	  }
}

/**
  * do Cylinder Segmentation Rgb
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::doCylinderSegmentationRgb(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRgbIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudRgbIn = in;

	pcl::PointCloud<PointXYZRGB>::Ptr cloud_cylinder (new pcl::PointCloud<PointXYZRGB> ());

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	getNormalsFromCloudRgb(*cloudRgbIn, *cloud_normals);

	pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

	pcl::ExtractIndices<PointXYZRGB> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;

	pcl::SACSegmentationFromNormals<PointXYZRGB, pcl::Normal> seg;

	// Create the segmentation object for cylinder segmentation and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_CYLINDER);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight (0.1);
	seg.setMaxIterations (5000);
	seg.setDistanceThreshold (0.05);
	seg.setRadiusLimits (0.2, 0.6);
	seg.setInputCloud (cloudRgbIn);
	seg.setInputNormals (cloud_normals);

	// Obtain the cylinder inliers and coefficients
	seg.segment (*inliers_cylinder, *coefficients_cylinder);
	std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

	// Write the cylinder inliers to disk
	extract.setInputCloud (cloudRgbIn);
	extract.setIndices (inliers_cylinder);
	extract.setNegative (false);
	//pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
	extract.filter (*cloud_cylinder);
	if (cloud_cylinder->points.empty ())
		std::cerr << "Can't find the cylindrical component." << std::endl;
	else{
		std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->size () << " data points." << std::endl;
		//writer.write ("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
	}
	out = *cloud_cylinder;
}

/**
  * narfKeypointExtraction
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZ>& out
  * \return	void
  *
  */
void pclUtilitys::narfKeypointExtraction(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointXYZ>& out){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	*cloudIn = in;

	//Parameter
	float angular_resolution = 0.2f;
	float support_size = 0.02f;//0.2
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	bool setUnseenToMaxRange = true;
	//////////////////
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (in.sensor_origin_[0],in.sensor_origin_[1],in.sensor_origin_[2])) * Eigen::Affine3f (in.sensor_orientation_);

    // -----------------------------------------------
    // -----Create RangeImage from the PointCloud-----
    // -----------------------------------------------
    float noise_level = 0.1f;
    float min_range = 0.0f;
    int border_size = 1;
    pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;
    range_image.createFromPointCloud (in, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                     scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    range_image.integrateFarRanges (far_ranges);
    if (setUnseenToMaxRange)
      range_image.setUnseenToMaxRange ();

    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewerNarf ("3D Viewer NARF");
    viewerNarf.setBackgroundColor (1.0f, 1.0f, 1.0f);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0.0f, 0.0f, 0.0f);
    viewerNarf.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
    viewerNarf.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");

    viewerNarf.initCameraParameters ();
    //setViewerPose (viewerNarf, range_image.getTransformationToWorldSystem ());

    // --------------------------
    // -----Show range image-----
    // --------------------------
    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    if(range_image.empty()){cout << "range_image is empty"<< endl;}
    range_image_widget.showRangeImage (range_image);

    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage (&range_image);
    narf_keypoint_detector.getParameters ().support_size = support_size;
    narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
    narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;

    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute (keypoint_indices);
    std::cout << "Found "<<keypoint_indices.size ()<<" key points.\n";

    // ----------------------------------------------
    // -----Show keypoints in range image widget-----
    // ----------------------------------------------
    //for (std::size_t i=0; i<keypoint_indices.size (); ++i)
      //range_image_widget.markPoint (keypoint_indices[i]%range_image.width,
                                    //keypoint_indices[i]/range_image.width);

    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>& keypoints = *keypoints_ptr;
    keypoints.resize (keypoint_indices.size ());
    for (std::size_t i=0; i<keypoint_indices.size (); ++i)
      keypoints[i].getVector3fMap () = range_image[keypoint_indices[i]].getVector3fMap ();
/*
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (keypoints_ptr, 0, 255, 0);
    viewerNarf.addPointCloud<pcl::PointXYZ> (keypoints_ptr, keypoints_color_handler, "keypoints");
    viewerNarf.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
*/
    out = *keypoints_ptr;
    /*
    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewerNarf.wasStopped ())
    {
      range_image_widget.spinOnce ();  // process GUI events
      viewerNarf.spinOnce ();
      pcl_sleep(0.01);
    }
    */
}

/**
  * narfKeypointExtraction RGB
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::narfKeypointExtractionRgb(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = in;

	//Parameter
	float angular_resolution = 0.008f;
	float support_size = 0.01f;//0.2
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	bool setUnseenToMaxRange = true;
	//////////////////
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (in.sensor_origin_[0],in.sensor_origin_[1],in.sensor_origin_[2])) * Eigen::Affine3f (in.sensor_orientation_);

    // -----------------------------------------------
    // -----Create RangeImage from the PointCloud-----
    // -----------------------------------------------
    float noise_level = 0.05f;
    float min_range = 0.02f;
    int border_size = 1;
    pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;
    //range_image = &range_image_ptr;
    range_image.createFromPointCloud (in, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                     scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    range_image.integrateFarRanges (far_ranges);
    if (setUnseenToMaxRange)
      range_image.setUnseenToMaxRange ();
    /*
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewerNarf ("3D Viewer NARF");
    viewerNarf.setBackgroundColor (1.0f, 1.0f, 1.0f);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0.0f, 0.0f, 0.0f);
    viewerNarf.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
    viewerNarf.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");

    viewerNarf.initCameraParameters ();
    //setViewerPose (viewerNarf, range_image.getTransformationToWorldSystem ());

    // --------------------------
    // -----Show range image-----
    // --------------------------
    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    if(range_image.empty()){cout << "range_image is empty"<< endl;}
    range_image_widget.showRangeImage (range_image);
*/
    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage (&range_image);
    narf_keypoint_detector.getParameters ().support_size = support_size;
    narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
    narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;

    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute (keypoint_indices);
    std::cout << "Found "<<keypoint_indices.size ()<<" key points.\n";

    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>& keypoints = *keypoints_ptr;
    keypoints.resize (keypoint_indices.size ());
    for (std::size_t i=0; i<keypoint_indices.size (); ++i)
      keypoints[i].getVector3fMap () = range_image[keypoint_indices[i]].getVector3fMap ();

    out = *keypoints_ptr;
}

/**
  * merge two PCL without color
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& in1, const pcl::PointCloud<pcl::PointXYZ>& in2
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZ>::Ptr& out
  * \return	void
  *
  */
void pclUtilitys::mergeXYZPcl(const pcl::PointCloud<pcl::PointXYZ>& in1, const pcl::PointCloud<pcl::PointXYZ>& in2, pcl::PointCloud<pcl::PointXYZ>::Ptr& out){
	int i=0;
	int imax = in1.points.size();
	//cout << "imax in1: " << imax << endl;
	while(i<imax){
		out->push_back(in1.points[i]);
	    i++;
	}
	i=0;
	imax = in2.points.size();
	cout << "imax in2: " << imax << endl;
	while(i<imax){		
		out->push_back(in2.points[i]);
		i++;
	}	
}

/**
  * merge two PCL with color
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in1, const pcl::PointCloud<pcl::PointXYZRGB>& in2
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out
  * \return	void
  *
  */
void pclUtilitys::mergeXYZRGBPcl(const pcl::PointCloud<pcl::PointXYZRGB>& in1, const pcl::PointCloud<pcl::PointXYZRGB>& in2, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out){
	//long int outPclSize = in1.size()+in2.size();
	//vector<pcl::PointXYZRGB> outPcl;
	//out->points.resize(outPclSize);
	int i=0;
	int imax = in1.points.size();
	cout << "imax in1: " << imax << endl;
	while(i<imax){
		out->push_back(in1.points[i]);
		/*
	    out->points[i].r = 0.0;
	    out->points[i].g = 0.0;
	    out->points[i].b = 0.0;		
		*/
	    i++;
	}
	i=0;
	imax = in2.points.size();
	cout << "imax in2: " << imax << endl;
	while(i<imax){		
		out->push_back(in2.points[i]);
		/*
		out->points[j].r = 0.0;
	    out->points[j].g = 1.0;
	    out->points[j].b = 0.0;		
		*/
		i++;
	}	
}

/**
  * ICP simple
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in1, const pcl::PointCloud<pcl::PointXYZRGB>& in2
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out
  * \return	void
  *
  */
void pclUtilitys::icpSimple(const pcl::PointCloud<pcl::PointXYZRGB>& in1, const pcl::PointCloud<pcl::PointXYZRGB>& in2, pcl::PointCloud<pcl::PointXYZRGB>& out){
	int iterations = 20000;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbIn1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbIn2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	*rgbIn1 = in1;
	*rgbIn2 = in2;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr pclIn1 (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr pclIn2 (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbOut (new pcl::PointCloud<pcl::PointXYZRGB>);
	//////////////////////////////
	//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//convertXYZRGBtoXYZ(*rgbIn1, pclIn1);
	//convertXYZRGBtoXYZ(*rgbIn2, pclIn2);
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(rgbIn1);
	icp.setInputTarget(rgbIn2);	
 	icp.setMaximumIterations (iterations);
	icp.setTransformationEpsilon (1e-6);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
  	// Note: adjust this based on the size of your datasets
  	icp.setMaxCorrespondenceDistance (0.01);
	/*
		icp.setPointRepresentation (pcl::make_shared<const MyPointRepresentation> (point_representation));
		// Instantiate our custom point representation (defined above) ...
		MyPointRepresentation point_representation;
		// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
		float alpha[4] = {1.0, 1.0, 1.0, 1.0};
		point_representation.setRescaleValues (alpha);
	*/

	//pcl::PointCloud<pcl::PointXYZ> final;
	icp.align(out);
	//convertPtrXYZtoXYZRGB(final, rgbOut);
	//out = *rgbOut;
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	transM = icp.getFinalTransformation();
	//std::cout << "final Transformation...\n" << icp.getFinalTransformation() << std::endl;
}

/**
  * ICP via normals
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in1, const pcl::PointCloud<pcl::PointXYZRGB>& in2
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out
  * \return	void
  *
  */
void pclUtilitys::icpNormals(const pcl::PointCloud<pcl::PointXYZRGB>& in1, const pcl::PointCloud<pcl::PointXYZRGB>& in2, pcl::PointCloud<pcl::PointXYZRGB>& out){
	int iterations = 20000;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbIn1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbIn2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outResult (new pcl::PointCloud<pcl::PointXYZRGB>);
	*rgbIn1 = in1;
	*rgbIn2 = in2;
	*outResult = out;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgbNormal1 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rgbNormal2 (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	pclUtilitys::convertXYZRGBtoXYZRGBNormal(*rgbIn1, rgbNormal1);
	pclUtilitys::convertXYZRGBtoXYZRGBNormal(*rgbIn2, rgbNormal2);

	pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
	icp.setInputSource(rgbNormal1);
	icp.setInputTarget(rgbNormal2);	
 	icp.setMaximumIterations (iterations);
	icp.setTransformationEpsilon (1e-6);
	icp.setMaxCorrespondenceDistance (0.001);
	/*
	MyPointRepresentation point_representation;
	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  	point_representation.setRescaleValues (alpha);
	icp.setPointRepresentation (pcl::make_shared<const pcl::PointXYZRGBNormal> (point_representation));
	*/
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr icpAligned (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	icp.align(*icpAligned);
	transM = icp.getFinalTransformation();

	pclUtilitys::convertXYZRGBNormaltoXYZRGB(*icpAligned, outResult);
	out = *outResult;
}

/**
  * rotate PCL over X axis
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in, float radiantAngle
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out,
  * \return	void
  *
  */
void pclUtilitys::rotateX(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out, float radiantAngle){	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = in;
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	transform_2.rotate (Eigen::AngleAxisf (radiantAngle, Eigen::Vector3f::UnitX()));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::transformPointCloud<pcl::PointXYZRGB> (*cloudIn, out, transform_2);
}
/**
  * rotate PCL over Y axis
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in, float radiantAngle
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out,
  * \return	void
  *
  */
void pclUtilitys::rotateY(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out, float radiantAngle){	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = in;
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	transform_2.rotate (Eigen::AngleAxisf (radiantAngle, Eigen::Vector3f::UnitY()));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::transformPointCloud<pcl::PointXYZRGB> (*cloudIn, out, transform_2);
}

/**
  * rotate PCL over Z axis
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in, float radiantAngle
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out,
  * \return	void
  *
  */
void pclUtilitys::rotateZ(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out, float radiantAngle){	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = in;
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	transform_2.rotate (Eigen::AngleAxisf (radiantAngle, Eigen::Vector3f::UnitZ()));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::transformPointCloud<pcl::PointXYZRGB> (*cloudIn, out, transform_2);
}

/**
  * rotate PCL based in euler angle and in XYZ steps
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in, float x,y,z euler angle in deg
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out,
  * \return	void
  *
  */
void pclUtilitys::rotateEulerXYZ(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out,const float xAngle, const float yAngle,const float zAngle){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = in;
	Eigen::Affine3f transformIt = Eigen::Affine3f::Identity();
	transformIt.rotate (Eigen::AngleAxisf (xAngle, Eigen::Vector3f::UnitX()));
	transformIt.rotate (Eigen::AngleAxisf (yAngle, Eigen::Vector3f::UnitY()));
	transformIt.rotate (Eigen::AngleAxisf (zAngle, Eigen::Vector3f::UnitZ()));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::transformPointCloud<pcl::PointXYZRGB> (*cloudIn, out, transformIt);	
}

/**
  * rotate PCL based in euler angle and in XYZ steps
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in, float x,y,z euler angle in deg
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out,
  * \return	void
  *
  */
void pclUtilitys::rotateEulerYZX(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out,const float yAngle, const float zAngle,const float xAngle){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = in;
	Eigen::Affine3f transformIt = Eigen::Affine3f::Identity();
	transformIt.rotate (Eigen::AngleAxisf (yAngle, Eigen::Vector3f::UnitY()));
	transformIt.rotate (Eigen::AngleAxisf (zAngle, Eigen::Vector3f::UnitZ()));
	transformIt.rotate (Eigen::AngleAxisf (xAngle, Eigen::Vector3f::UnitX()));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::transformPointCloud<pcl::PointXYZRGB> (*cloudIn, out, transformIt);	
}

/**
  * keypointExtractionNARF
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZ>& out
  * \return	void
  *
  */
void pclUtilitys::keypointExtractionNARF(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointXYZ>& out){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	*cloudIn = in;
	//PointCloud<PointWithScale>::Ptr pSift(new PointCloud<PointWithScale>());
	//PointCloud<PointT>::Ptr pSift(new PointCloud<PointT>());
	PointCloud<PointXYZ>::Ptr pSift(new PointCloud<PointXYZ>());	
	DetectNARFKeypoints(cloudIn, pSift);
	std::cout << "NARF KeyPoints: " << pSift->points.size() << std::endl;
	pcl::copyPointCloud(*pSift, out);
}

/**
  * keypointExtractionNARF
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointWithScale>& out
  * \return	void
  *
  */
void pclUtilitys::keypointExtractionNARF(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointWithScale>& out){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	*cloudIn = in;
	//PointCloud<PointWithScale>::Ptr pSift(new PointCloud<PointWithScale>());
	//PointCloud<PointT>::Ptr pSift(new PointCloud<PointT>());
	PointCloud<PointXYZ>::Ptr pSift(new PointCloud<PointXYZ>());
	DetectNARFKeypoints(cloudIn, pSift);
	std::cout << "NARF KeyPoints: " << pSift->points.size() << std::endl;
	pcl::copyPointCloud(*pSift, out);	
}

/**
  * keypointExtractionISS
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::keypointExtractionISS(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn = in;
	/*
		const std::string strFilePath = std::string("/media/hd2/db/RGBD/kv2/400/0_020298740747_PCLrgb.ply");
	    pcl::PLYReader Reader;
        Reader.read(strFilePath, *cloudIn);
	*/
	PointCloud<PointXYZRGB>::Ptr pIss(new PointCloud<PointXYZRGB>());
	DetectISSKeypoints(cloudIn, pIss);
	std::cout << "ISS KeyPoints: " << pIss->points.size() << std::endl;
	pcl::copyPointCloud(*pIss, out);
}

/**
  * keypointExtractionSIFT
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::keypointExtractionSIFT(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints (new pcl::PointCloud<pcl::PointWithScale>);
	*cloudIn = in;
        // Estimate the normals
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> normalEstimation;
        pcl::PointCloud<pcl::PointNormal>::Ptr normalCloud (new pcl::PointCloud<pcl::PointNormal>);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr normalKdTree(new pcl::search::KdTree<pcl::PointXYZRGB>());

        normalEstimation.setInputCloud(cloudIn);
        normalEstimation.setSearchMethod(normalKdTree);
        normalEstimation.setRadiusSearch(0.02);
        normalEstimation.compute(*normalCloud);

        for(size_t i = 0; i<normalCloud->points.size(); ++i)
        {
            normalCloud->points[i].x = cloudIn->points[i].x;
            normalCloud->points[i].y = cloudIn->points[i].y;
            normalCloud->points[i].z = cloudIn->points[i].z;
        }

        float minScale = 0.002f;// the standard deviation of the smallest scale in the scale space
        int nrOctaves = 4;// the number of octaves (i.e. doublings of scale) to compute
        int nrScalesPerOctave = 4;// the number of scales to compute within each octave
        float minContrast = 0.001f;// the minimum contrast required for detection

        SIFTKeypoint<PointNormal, PointWithScale> siftDetect;
        search::KdTree<PointNormal>::Ptr kdTree(new search::KdTree<PointNormal>());
        siftDetect.setSearchMethod(kdTree);
        siftDetect.setScales(minScale, nrOctaves, nrScalesPerOctave);
        siftDetect.setMinimumContrast(minContrast);
        siftDetect.setInputCloud(normalCloud);
        siftDetect.compute(*keypoints);	
		pcl::copyPointCloud(*keypoints, out);
}

/**
  * keypointExtractionSIFT
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointWithScale>& out
  * \return	void
  *
  */
void pclUtilitys::keypointExtractionSIFT(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointWithScale>& out){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints (new pcl::PointCloud<pcl::PointWithScale>);
	*cloudIn = in;
        // Estimate the normals
        pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normalEstimation;
        pcl::PointCloud<pcl::PointNormal>::Ptr normalCloud (new pcl::PointCloud<pcl::PointNormal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr normalKdTree(new pcl::search::KdTree<pcl::PointXYZ>());

        normalEstimation.setInputCloud(cloudIn);
        normalEstimation.setSearchMethod(normalKdTree);
        normalEstimation.setRadiusSearch(0.02);
        normalEstimation.compute(*normalCloud);

        for(size_t i = 0; i<normalCloud->points.size(); ++i)
        {
            normalCloud->points[i].x = cloudIn->points[i].x;
            normalCloud->points[i].y = cloudIn->points[i].y;
            normalCloud->points[i].z = cloudIn->points[i].z;
        }

        float minScale = 0.002f;// the standard deviation of the smallest scale in the scale space
        int nrOctaves = 4;// the number of octaves (i.e. doublings of scale) to compute
        int nrScalesPerOctave = 4;// the number of scales to compute within each octave
        float minContrast = 0.001f;// the minimum contrast required for detection

        SIFTKeypoint<PointNormal, PointWithScale> sift;
        search::KdTree<PointNormal>::Ptr kdTree(new search::KdTree<PointNormal>());
        sift.setSearchMethod(kdTree);
        sift.setScales(minScale, nrOctaves, nrScalesPerOctave);
        sift.setMinimumContrast(minContrast);
        sift.setInputCloud(normalCloud);
        sift.compute(*keypoints);	
		pcl::copyPointCloud(*keypoints, out);
}

/**
  * keypointExtractionSIFT
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointNormal>::Ptr& normalCloud, pcl::PointCloud<pcl::PointWithScale>& out
  * \return	void
  *
  */
void pclUtilitys::keypointExtractionSIFT(const pcl::PointCloud<pcl::PointXYZ>& in, pcl::PointCloud<pcl::PointNormal>::Ptr& normalCloud, pcl::PointCloud<pcl::PointWithScale>& out){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints (new pcl::PointCloud<pcl::PointWithScale>);
	*cloudIn = in;

		int i=0;
		int imax = normalCloud->points.size();
		while(i<imax)        {
            normalCloud->points[i].x = cloudIn->points[i].x;
            normalCloud->points[i].y = cloudIn->points[i].y;
            normalCloud->points[i].z = cloudIn->points[i].z;
			i++;
        }

        float minScale = 0.002f;// the standard deviation of the smallest scale in the scale space
        int nrOctaves = 8;// the number of octaves (i.e. doublings of scale) to compute
        int nrScalesPerOctave = 8;// the number of scales to compute within each octave
        float minContrast = 0.001f;// the minimum contrast required for detection

        SIFTKeypoint<PointNormal, PointWithScale> sift;
        search::KdTree<PointNormal>::Ptr kdTree(new search::KdTree<PointNormal>());
        sift.setSearchMethod(kdTree);
        sift.setScales(minScale, nrOctaves, nrScalesPerOctave);
        sift.setMinimumContrast(minContrast);
        sift.setInputCloud(normalCloud);
        sift.compute(*keypoints);	
		pcl::copyPointCloud(*keypoints, out);
}

/**
  * keypointExtractionHarris
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::keypointExtractionHarris(const pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZRGB>& out){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);
	*cloudIn = in;
	float threshold = 0.0000001f;
	HarrisKeypoint3D<PointXYZRGB, PointXYZI> harrisDetect;
	harrisDetect.setNonMaxSupression(true);
	harrisDetect.setInputCloud(cloudIn);
	harrisDetect.setThreshold(threshold);
	harrisDetect.compute(*keypoints);	
	pcl::copyPointCloud(*keypoints, out);
}

/**
  * FastPointFeatureHistogramsExample
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZRGB>& in1, const pcl::PointCloud<pcl::PointXYZRGB>& in2,
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZRGB>& out
  * \return	void
  *
  */
void pclUtilitys::FastPointFeatureHistogramsExample(const pcl::PointCloud<pcl::PointXYZRGB>& in1, const pcl::PointCloud<pcl::PointXYZRGB>& in2, pcl::PointCloud<pcl::PointXYZRGB>& out){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn1 = in1;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloudIn2 = in2;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pointcloud::Ptr source(new pointcloud);
	pointcloud::Ptr target(new pointcloud);
	pcl::copyPointCloud(*cloudIn1, *source);
	pcl::copyPointCloud(*cloudIn2, *target);

    float s1 = 0.02, s2 = 0.02; //default = 0.01

    pointcloud::Ptr source_f(new pointcloud);
    pcl::console::print_info("Waiting for filtering the data．．．\n");
    pcl::VoxelGrid<pcl::PointXYZ> sor; 
    sor.setInputCloud(source);
    sor.setLeafSize(s1, s1, s1);
    sor.filter(*source_f); 

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_f(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::console::print_info("Waiting for filtering the data．．．\n");
    pcl::VoxelGrid<pcl::PointXYZ> sor2;
    sor2.setInputCloud(target);
    sor2.setLeafSize(s2, s2, s2);
    sor2.filter(*target_f); 


 	fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source_f, tree);
    //cout << "calculate time is: " << float(end3 - start) / CLOCKS_PER_SEC << endl;
    cout << "source cloud number is " << source_f->points.size() << endl;

    fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target_f, tree);
    //cout << "calculate time is: " << float(end2 - end3) / CLOCKS_PER_SEC << endl;
    cout << "targrt cloud number is " << target_f->points.size() << endl;


    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setMaximumIterations(60);
    sac_ia.setInputSource(source_f);
    sac_ia.setSourceFeatures(source_fpfh);
    sac_ia.setInputTarget(target_f);
    sac_ia.setTargetFeatures(target_fpfh);
    pointcloud::Ptr align(new pointcloud);
	sac_ia.setNumberOfSamples(30); 
    sac_ia.align(*align);

    //cout << "calculate time is: " << float(end - end2) / CLOCKS_PER_SEC << endl;
	pcl::copyPointCloud(*align, out);
 
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> alignColorHandler(align, 255, 255, 0);
    viewer.addPointCloud(align, alignColorHandler, "align");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "align");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sourceColorHandler(source, 0, 0, 200);
    viewer.addPointCloud(source, sourceColorHandler, "source");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> targetColorHandler(target, 120, 120, 120);
    viewer.addPointCloud(target, targetColorHandler, "target");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

/**
  * extractFPFHfromKeypoints
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& in1, const pcl::PointCloud<pcl::PointXYZ>& in2,
                                           const pcl::PointCloud<pcl::PointWithScale>& inKP1, const pcl::PointCloud<pcl::PointWithScale>& inKP2,
										   const pcl::PointCloud<pcl::PointNormal>& inN1, const pcl::PointCloud<pcl::PointNormal>& inN2,
  * output:\n
  * \param	pcl::PointCloud<pcl::PointXYZ>& out
  * \return	void
  *
  */
void pclUtilitys::extractFPFHfromKeypoints(const pcl::PointCloud<pcl::PointXYZ>& in1, const pcl::PointCloud<pcl::PointXYZ>& in2,
                                           const pcl::PointCloud<pcl::PointWithScale>& inKP1, const pcl::PointCloud<pcl::PointWithScale>& inKP2,
										   const pcl::PointCloud<pcl::PointNormal>& inN1, const pcl::PointCloud<pcl::PointNormal>& inN2,
										    pcl::PointCloud<pcl::PointXYZ>& out){
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	*source_cloud_ptr = in1;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	*target_cloud_ptr = in2;	

	pcl::PointCloud<pcl::PointWithScale>::Ptr src_keypoints (new pcl::PointCloud<pcl::PointWithScale>);
	*src_keypoints = inKP1;	
	pcl::PointCloud<pcl::PointWithScale>::Ptr tar_keypoints (new pcl::PointCloud<pcl::PointWithScale>);
	*tar_keypoints = inKP2;	

	pcl::PointCloud<pcl::PointNormal>::Ptr src_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
	*src_normals_ptr = inN1;	
	pcl::PointCloud<pcl::PointNormal>::Ptr tar_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
	*tar_normals_ptr = inN2;		

  // Extract FPFH features from SIFT keypoints
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZ>);                           
  pcl::copyPointCloud (*src_keypoints, *src_keypoints_xyz);
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
  fpfh.setSearchSurface (source_cloud_ptr);
  fpfh.setInputCloud (src_keypoints_xyz);
  fpfh.setInputNormals (src_normals_ptr);
  fpfh.setSearchMethod (pclUtilitys::tree_xyz);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
  pcl::PointCloud<pcl::FPFHSignature33>& src_features = *src_features_ptr;
  fpfh.setRadiusSearch(0.05);
  fpfh.compute(src_features);
  cout << "Computed " << src_features.size() << " FPFH features for source cloud\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr tar_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZ>);                           
  pcl::copyPointCloud (*tar_keypoints, *tar_keypoints_xyz);
  fpfh.setSearchSurface (target_cloud_ptr);
  fpfh.setInputCloud (tar_keypoints_xyz);
  fpfh.setInputNormals (tar_normals_ptr);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr tar_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
  pcl::PointCloud<pcl::FPFHSignature33>& tar_features = *tar_features_ptr;
  fpfh.compute(tar_features);
  cout << "Computed " << tar_features.size() << " FPFH features for target cloud\n";
  
  // Sample Consensus Initial Alignment parameters
const float min_sample_dist = 0.04f;
const float max_correspondence_dist = 0.02f;
const int nr_iters = 500;

// ICP parameters
const float max_correspondence_distance = 0.02f;
const float outlier_rejection_threshold = 0.02f;
const float transformation_epsilon = 0;
const int max_iterations = 200;

  // Compute the transformation matrix for alignment
  Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
  tform = computeInitialAlignment (src_keypoints, src_features_ptr, tar_keypoints,
          tar_features_ptr, min_sample_dist, max_correspondence_dist, nr_iters);
  
  /* Uncomment this code to run ICP */
  tform = refineAlignment (source_cloud_ptr, target_cloud_ptr, tform, max_correspondence_distance,
          outlier_rejection_threshold, transformation_epsilon, max_iterations);
  
  //pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>& transformed_cloud = *transformed_cloud_ptr;
  pcl::transformPointCloud(in1, out, tform);	
}

/**
  * extractFPFHfromKeypoints
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& in1, const pcl::PointCloud<pcl::PointXYZ>& in2,
										const pcl::PointCloud<pcl::PointWithScale>& inKP1, const pcl::PointCloud<pcl::PointWithScale>& inKP2,
										const pcl::PointCloud<pcl::PointNormal>& inN1, const pcl::PointCloud<pcl::PointNormal>& inN2,
  * output:\n
  * \param	Eigen::Matrix4f& tform
  * \return	void
  *
  */
void pclUtilitys::extractFPFHfromKeypoints(const pcl::PointCloud<pcl::PointXYZ>& in1, const pcl::PointCloud<pcl::PointXYZ>& in2,
										const pcl::PointCloud<pcl::PointWithScale>& inKP1, const pcl::PointCloud<pcl::PointWithScale>& inKP2,
										const pcl::PointCloud<pcl::PointNormal>& inN1, const pcl::PointCloud<pcl::PointNormal>& inN2,
										Eigen::Matrix4f& tform){
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	*source_cloud_ptr = in1;
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	*target_cloud_ptr = in2;	

	pcl::PointCloud<pcl::PointWithScale>::Ptr src_keypoints (new pcl::PointCloud<pcl::PointWithScale>);
	*src_keypoints = inKP1;	
	pcl::PointCloud<pcl::PointWithScale>::Ptr tar_keypoints (new pcl::PointCloud<pcl::PointWithScale>);
	*tar_keypoints = inKP2;	

	pcl::PointCloud<pcl::PointNormal>::Ptr src_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
	*src_normals_ptr = inN1;	
	pcl::PointCloud<pcl::PointNormal>::Ptr tar_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
	*tar_normals_ptr = inN2;		

  // Extract FPFH features from SIFT keypoints
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZ>);                           
  pcl::copyPointCloud (*src_keypoints, *src_keypoints_xyz);
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
  fpfh.setSearchSurface (source_cloud_ptr);
  fpfh.setInputCloud (src_keypoints_xyz);
  fpfh.setInputNormals (src_normals_ptr);
  fpfh.setSearchMethod (pclUtilitys::tree_xyz);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
  pcl::PointCloud<pcl::FPFHSignature33>& src_features = *src_features_ptr;
  fpfh.setRadiusSearch(0.05);
  fpfh.compute(src_features);
  cout << "Computed " << src_features.size() << " FPFH features for source cloud\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr tar_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZ>);                           
  pcl::copyPointCloud (*tar_keypoints, *tar_keypoints_xyz);
  fpfh.setSearchSurface (target_cloud_ptr);
  fpfh.setInputCloud (tar_keypoints_xyz);
  fpfh.setInputNormals (tar_normals_ptr);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr tar_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
  pcl::PointCloud<pcl::FPFHSignature33>& tar_features = *tar_features_ptr;
  fpfh.compute(tar_features);
  cout << "Computed " << tar_features.size() << " FPFH features for target cloud\n";
  
  // Sample Consensus Initial Alignment parameters (explanation below)
const float min_sample_dist = 0.02f;
const float max_correspondence_dist = 0.041f;
const int nr_iters = 500;

// ICP parameters (explanation below)
const float max_correspondence_distance = 0.10f;
const float outlier_rejection_threshold = 0.041f;
const float transformation_epsilon = 0;
const int max_iterations = 200;

  // Compute the transformation matrix for alignment
  tform = computeInitialAlignment (src_keypoints, src_features_ptr, tar_keypoints,
          tar_features_ptr, min_sample_dist, max_correspondence_dist, nr_iters);
  
  /* Uncomment this code to run ICP */
  tform = refineAlignment (source_cloud_ptr, target_cloud_ptr, tform, max_correspondence_distance,
          outlier_rejection_threshold, transformation_epsilon, max_iterations);								
}

/**
  * reconTwoPcl
  * input:\n
  * \param	const pcl::PointCloud<pcl::PointXYZ>& in1, const pcl::PointCloud<pcl::PointXYZ>& in2
  * output:\n
  * \param	Eigen::Matrix4f& tform
  * \return	void
  *
  */
void pclUtilitys::reconTwoPcl(const pcl::PointCloud<pcl::PointXYZ>& in1, const pcl::PointCloud<pcl::PointXYZ>& in2, Eigen::Matrix4f& tform){
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclfilter1 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclfilter2 (new pcl::PointCloud<pcl::PointXYZ>);

	voxelGrid(in1, *pclfilter1, 0.020f);
	voxelGrid(in2, *pclfilter2, 0.020f);

	pcl::PointCloud<pcl::PointNormal>::Ptr src_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr tar_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
	getNormalsFromCloud(*pclfilter1, *src_normals_ptr);
	getNormalsFromCloud(*pclfilter2, *tar_normals_ptr);

	pcl::PointCloud<pcl::PointWithScale>::Ptr src_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
	keypointExtractionSIFT(*pclfilter1, src_normals_ptr, *src_keypoints_ptr);	//voxel grid error - why ever
	pcl::PointCloud<pcl::PointWithScale>::Ptr tar_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
	keypointExtractionSIFT(*pclfilter2, tar_normals_ptr, *tar_keypoints_ptr);	//voxel grid error - why ever
	//Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
  	extractFPFHfromKeypoints(*pclfilter1, *pclfilter2,
                                     *src_keypoints_ptr, *tar_keypoints_ptr,
									 *src_normals_ptr, *tar_normals_ptr, tform);
}

////////////////////////////////////////////////////
// Private functions
///////////////////////////////////////
/**
  * computeInitialAlignment
  * input:\n
  * \param	const PointCloudPtr & source_points, const LocalDescriptorsPtr & source_descriptors,
                         const PointCloudPtr & target_points, const LocalDescriptorsPtr & target_descriptors,
                         float min_sample_distance, float max_correspondence_distance, int nr_iterations
  * \return	Eigen::Matrix4f 
  *
  */
Eigen::Matrix4f pclUtilitys::computeInitialAlignment (const PointCloudPtr & source_points, const LocalDescriptorsPtr & source_descriptors,
                         const PointCloudPtr & target_points, const LocalDescriptorsPtr & target_descriptors,
                         float min_sample_distance, float max_correspondence_distance, int nr_iterations)
{
  pcl::SampleConsensusInitialAlignment<PointT, PointT, LocalDescriptorT> sac_ia;
  sac_ia.setMinSampleDistance (min_sample_distance);
  sac_ia.setMaxCorrespondenceDistance (max_correspondence_distance);
  sac_ia.setMaximumIterations (nr_iterations);

  sac_ia.setInputSource (source_points);
  sac_ia.setSourceFeatures (source_descriptors);

  sac_ia.setInputTarget (target_points);
  sac_ia.setTargetFeatures (target_descriptors);

  pcl::PointCloud<PointT> registration_output;
  sac_ia.align (registration_output);

  return (sac_ia.getFinalTransformation ());
}

/**
  * refineAlignment
  * input:\n
  * \param	const ICPPointCloudPtr & source_points, const ICPPointCloudPtr & target_points,
                 const Eigen::Matrix4f initial_alignment, float max_correspondence_distance,
                 float outlier_rejection_threshold, float transformation_epsilon, float max_iterations
  * \return	Eigen::Matrix4f 
  *
  */
Eigen::Matrix4f pclUtilitys::refineAlignment (const ICPPointCloudPtr & source_points, const ICPPointCloudPtr & target_points,
                 const Eigen::Matrix4f initial_alignment, float max_correspondence_distance,
                 float outlier_rejection_threshold, float transformation_epsilon, float max_iterations) {

  pcl::IterativeClosestPoint<ICPPointT, ICPPointT> icp;
  icp.setMaxCorrespondenceDistance (max_correspondence_distance);
  icp.setRANSACOutlierRejectionThreshold (outlier_rejection_threshold);
  icp.setTransformationEpsilon (transformation_epsilon);
  icp.setMaximumIterations (max_iterations);

  ICPPointCloudPtr source_points_transformed (new ICPPointCloud);
  pcl::transformPointCloud (*source_points, *source_points_transformed, initial_alignment);

  icp.setInputSource (source_points_transformed);
  icp.setInputTarget (target_points);

  ICPPointCloud registration_output;
  icp.align (registration_output);

  return (icp.getFinalTransformation () * initial_alignment);
}

/**
  * DetectSIFTKeypoints
  * input:\n
  * \param	PointCloud<PointXYZRGB>::Ptr &cloud
  * output:\n
  * \param	PointCloud<PointT>::Ptr &keypoints
  * \return	void
  *
  */
//void pclUtilitys::DetectSIFTKeypoints(PointCloud<PointXYZRGB>::Ptr &cloud, PointCloud<PointWithScale>::Ptr &keypoints)
void pclUtilitys::DetectSIFTKeypoints(PointCloud<PointXYZRGB>::Ptr &cloud, PointCloud<PointT>::Ptr &keypoints)
{
	// Estimate the normals
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> normalEstimation;
	pcl::PointCloud<pcl::PointNormal>::Ptr normalCloud (new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr normalKdTree(new pcl::search::KdTree<pcl::PointXYZRGB>());

	normalEstimation.setInputCloud(cloud);
	normalEstimation.setSearchMethod(normalKdTree);
	normalEstimation.setRadiusSearch(0.05);
	normalEstimation.compute(*normalCloud);

	for(size_t i = 0; i<normalCloud->points.size(); ++i)
	{
		normalCloud->points[i].x = cloud->points[i].x;
		normalCloud->points[i].y = cloud->points[i].y;
		normalCloud->points[i].z = cloud->points[i].z;
	}

	float minScale = 0.001f;
	int nrOctaves = 3;
	int nrScalesPerOctave = 4;
	float minContrast = 0.0001f;
	
	pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;

	//SIFTKeypoint<PointNormal, PointWithScale> sift;
	search::KdTree<PointNormal>::Ptr kdTree(new search::KdTree<PointNormal>());
	sift.setSearchMethod(kdTree);
	sift.setScales(minScale, nrOctaves, nrScalesPerOctave);
	sift.setMinimumContrast(minContrast);
	sift.setInputCloud(normalCloud);
	sift.compute(*keypoints);
}

/**
  * DetectHarrisKeypoints
  * input:\n
  * \param	PointCloud<PointXYZRGB>::Ptr &cloud
  * output:\n
  * \param	PointCloud<PointXYZI>::Ptr &keypoints
  * \return	void
  *
  */
void pclUtilitys::DetectHarrisKeypoints(PointCloud<PointXYZRGB>::Ptr &cloud, PointCloud<PointXYZI>::Ptr &keypoints)
{
	float threshold = 0.0000001f;
	HarrisKeypoint3D<PointXYZRGB, PointXYZI> harrisDetect;
	harrisDetect.setNonMaxSupression(true);
	harrisDetect.setInputCloud(cloud);
	harrisDetect.setThreshold(threshold);
	harrisDetect.compute(*keypoints);
}

/**
  * DetectISSKeypoints
  * input:\n
  * \param	PointCloud<PointXYZRGB>::Ptr &cloud
  * output:\n
  * \param	PointCloud<PointXYZRGB>::Ptr &keypoints
  * \return	void
  *
  */
void pclUtilitys::DetectISSKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
									pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints)
{
	double modelResolution = 0.001f;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> issDetector;
	issDetector.setSearchMethod (kdTree);
	issDetector.setSalientRadius (6 * modelResolution);
	issDetector.setNonMaxRadius (4 * modelResolution);
	issDetector.setThreshold21 (0.9);
	issDetector.setThreshold32 (0.9);
	issDetector.setMinNeighbors (5);
	issDetector.setNumberOfThreads (4);
	issDetector.setInputCloud (cloud);
	issDetector.compute (*keypoints);
}

/**
  * DetectNARFKeypoints
  * input:\n
  * \param	PointCloud<PointXYZ>::Ptr &cloud
  * output:\n
  * \param	PointCloud<PointXYZ>::Ptr &keypoints
  * \return	void
  *
  */
void pclUtilitys::DetectNARFKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &keypoints)
{
	shared_ptr<RangeImage> rangeImage(new RangeImage);
	pcl::PointCloud<PointXYZ>& pointCloud = *cloud;

	Eigen::Affine3f sceneSensorPose = Eigen::Affine3f(Eigen::Translation3f(pointCloud.sensor_origin_[0], pointCloud.sensor_origin_[1], pointCloud.sensor_origin_[2]))*Eigen::Affine3f (pointCloud.sensor_orientation_);
	rangeImage->createFromPointCloud (pointCloud, pcl::deg2rad (0.5f), pcl::deg2rad (360.0f), pcl::deg2rad (180.0f), sceneSensorPose, pcl::RangeImage::CAMERA_FRAME, 0.0, 0.0f, 1);
	rangeImage->setUnseenToMaxRange();

	pcl::RangeImageBorderExtractor rangeImageBorderExtractor;
	pcl::NarfKeypoint narfKeypointDetector;
	narfKeypointDetector.setRangeImageBorderExtractor (&rangeImageBorderExtractor);
	narfKeypointDetector.setRangeImage (rangeImage.get());
	narfKeypointDetector.getParameters().support_size = 0.02f;
	narfKeypointDetector.setRadiusSearch(0.004);

	pcl::PointCloud<int> keypointIndices;
	narfKeypointDetector.compute (keypointIndices);

	keypoints->points.resize(keypointIndices.points.size());
	for (size_t i=0; i<keypointIndices.points.size(); ++i)
	{
		keypoints->points[i].getVector3fMap () = rangeImage->points[keypointIndices.points[i]].getVector3fMap();
	}
}

/**
  * DetectNARFKeypoints
  * input:\n
  * \param	pcl::PointCloud<pcl::PointXYZ> ::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree
  * \return	pcl::PointCloud<pcl::FPFHSignature33>::Ptr
  *
  */
pcl::PointCloud<pcl::FPFHSignature33>::Ptr pclUtilitys::compute_fpfh_feature(pcl::PointCloud<pcl::PointXYZ> ::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree){
    pointnormal::Ptr point_normal(new pointnormal);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
    est_normal.setInputCloud(input_cloud);
    est_normal.setSearchMethod(tree);
    est_normal.setKSearch(10);
    est_normal.compute(*point_normal);
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
    est_fpfh.setNumberOfThreads(6);
    est_fpfh.setInputCloud(input_cloud);
    est_fpfh.setInputNormals(point_normal);
    est_fpfh.setSearchMethod(tree);
    est_fpfh.setKSearch(10);
    est_fpfh.compute(*fpfh);
    return fpfh;
}

/**
  * meshing via Poisson
  * input:\n
  * \param	
  * \return	void 
  *
  */

void pclUtilitys::meshingViaPoisson(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, std::string filePath, int pointNormalMode){
	pcl::PolygonMesh triangles;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	if(pointNormalMode == 0){
		pclUtilitys::getPointNormalsFromCloudRgb(*input, *cloud_with_normals);
	}else{
		pclUtilitys::getSmoothPointNormalsFromCloudRgb(*input, *cloud_with_normals);
	}

	int poissonDepth = 6;
	int solver_divide = 6;//8
	int iso_divide = 6;//8
	float point_weight = 4.0f; //4.0f
	float scale = 1.1f; // 1.1f

	Poisson<PointNormal> poisson;
	poisson.setDepth(poissonDepth); //default 8
	poisson.setScale(scale);        //default 1.1
	poisson.setSolverDivide (solver_divide);
	poisson.setIsoDivide (iso_divide);
	poisson.setPointWeight (point_weight);
	poisson.setInputCloud (cloud_with_normals);
	poisson.reconstruct (triangles);

	//pcl::io::saveOBJFile("/var/www/ramdev/meshPoisson.obj", triangles);
	//pcl::io::savePolygonFileSTL("/var/www/ramdev/meshPoisson.stl", triangles);
	pcl::io::savePolygonFileSTL(filePath, triangles);
	/*
[--depth <reconstruction depth>]
    This integer is the maximum depth of the tree that will be used for surface reconstruction. Running at depth d corresponds to solving on a voxel grid whose resolution is no larger than 2^d x 2^d x 2^d. Note that since the reconstructor adapts the octree to the sampling density, the specified reconstruction depth is only an upper bound.
    The default value for this parameter is 8. 
[--minDepth <adaptive octree depth>]
    This integer specifies the depth beyond depth the octree will be adapted. At coarser depths, the octree will be complete, containing all 2^d x 2^d x 2^d nodes.
    The default value for this parameter is 5. 
[--pointWeight <interpolation weight>]
    This floating point value specifies the importants that interpolation of the point samples is given in the formulation of the screened Poisson equation.
    The results of the original (unscreened) Poisson Reconstruction can be obtained by setting this value to 0.
    The default value for this parameter is 4. 
[--threads <number of processing threads>]
    This integer specifies the number of threads across which the reconstruction algorithm should be parallelized.
    The default value for this parameter is equal to the numer of (virtual) processors on the executing machine. 
[--scale <scale factor>]
    This floating point value specifies the ratio between the diameter of the cube used for reconstruction and the diameter of the samples' bounding cube.
    The default value is 1.1. 
[--solverDivide <solver subdivision depth>]
    This integer argument specifies the depth at which a block Gauss-Seidel solver is used to solve the Laplacian equation. Using this parameter helps reduce the memory overhead at the cost of a small increase in reconstruction time. (In practice, we have found that for reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly reduce the memory usage.)
    The default value is 8. 
[--isoDivide <iso-surface extraction subdivision depth>]
    This integer argument specifies the depth at which a block iso-surface extractor should be used to extract the iso-surface. Using this parameter helps reduce the memory overhead at the cost of a small increase in extraction time. (In practice, we have found that for reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly reduce the memory usage.)
    The default value is 8. 
[--samplesPerNode <minimum number of samples>]
    This floating point value specifies the minimum number of sample points that should fall within an octree node as the octree construction is adapted to sampling density. For noise-free samples, small values in the range [1.0 - 5.0] can be used. For more noisy samples, larger values in the range [15.0 - 20.0] may be needed to provide a smoother, noise-reduced, reconstruction.
    The default value is 1.0. 
*/	
}

/**
  * meshing via Poisson
  * input: pcl::PointCloud<pcl::PointXYZRGB\n
  * output: pcl::PolygonMesh
  * \param	int pointNormalMode :=  = 0 = getPointNormalsFromCloudRgb ; >= 1 = pclUtilitys::getSmoothPointNormalsFromCloudRgb
  * \return	void 
  *
  */

void pclUtilitys::meshingViaPoisson(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, pcl::PolygonMesh &triangles, int pointNormalMode){
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	if(pointNormalMode == 0){
		pclUtilitys::getPointNormalsFromCloudRgb(*input, *cloud_with_normals);
	}else{
		pclUtilitys::getSmoothPointNormalsFromCloudRgb(*input, *cloud_with_normals);
	}
	
	int poissonDepth = 8; //8
	int solver_divide = 6;//8
	int iso_divide = 6;//8
	float point_weight = 16.0f; //4.0f
	float scale = 1.1f; // 1.1f

	Poisson<PointNormal> poisson;
	poisson.setDepth(poissonDepth); //default 8
	poisson.setScale(scale);        //default 1.1
	poisson.setSolverDivide (solver_divide);
	poisson.setIsoDivide (iso_divide);
	poisson.setPointWeight (point_weight);
	poisson.setInputCloud (cloud_with_normals);
	poisson.reconstruct (triangles);


	//pcl::io::saveOBJFile("/var/www/ramdev/meshPoisson.obj", triangles);
	//pcl::io::savePolygonFileSTL("/var/www/ramdev/meshPoisson.stl", triangles);
	/*
[--depth <reconstruction depth>]
    This integer is the maximum depth of the tree that will be used for surface reconstruction. Running at depth d corresponds to solving on a voxel grid whose resolution is no larger than 2^d x 2^d x 2^d. Note that since the reconstructor adapts the octree to the sampling density, the specified reconstruction depth is only an upper bound.
    The default value for this parameter is 8. 
[--minDepth <adaptive octree depth>]
    This integer specifies the depth beyond depth the octree will be adapted. At coarser depths, the octree will be complete, containing all 2^d x 2^d x 2^d nodes.
    The default value for this parameter is 5. 
[--pointWeight <interpolation weight>]
    This floating point value specifies the importants that interpolation of the point samples is given in the formulation of the screened Poisson equation.
    The results of the original (unscreened) Poisson Reconstruction can be obtained by setting this value to 0.
    The default value for this parameter is 4. 
[--threads <number of processing threads>]
    This integer specifies the number of threads across which the reconstruction algorithm should be parallelized.
    The default value for this parameter is equal to the numer of (virtual) processors on the executing machine. 
[--scale <scale factor>]
    This floating point value specifies the ratio between the diameter of the cube used for reconstruction and the diameter of the samples' bounding cube.
    The default value is 1.1. 
[--solverDivide <solver subdivision depth>]
    This integer argument specifies the depth at which a block Gauss-Seidel solver is used to solve the Laplacian equation. Using this parameter helps reduce the memory overhead at the cost of a small increase in reconstruction time. (In practice, we have found that for reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly reduce the memory usage.)
    The default value is 8. 
[--isoDivide <iso-surface extraction subdivision depth>]
    This integer argument specifies the depth at which a block iso-surface extractor should be used to extract the iso-surface. Using this parameter helps reduce the memory overhead at the cost of a small increase in extraction time. (In practice, we have found that for reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly reduce the memory usage.)
    The default value is 8. 
[--samplesPerNode <minimum number of samples>]
    This floating point value specifies the minimum number of sample points that should fall within an octree node as the octree construction is adapted to sampling density. For noise-free samples, small values in the range [1.0 - 5.0] can be used. For more noisy samples, larger values in the range [15.0 - 20.0] may be needed to provide a smoother, noise-reduced, reconstruction.
    The default value is 1.0. 
*/	
}

/**
  * meshing via Fast Triangulation - greedy projection
  * input: const pcl::PointCloud<pcl::PointXYZRGB>::Ptr \n
  * output: pcl::PolygonMesh &triangles\n
  * \param	
  * \return	void 
  *
  */
void pclUtilitys::meshingViaFastTriangulation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, pcl::PolygonMesh &triangles, int pointNormalMode){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pclUtilitys::convertXYZRGBtoXYZ(*input, cloud);

	// Concatenate the XYZ and normal fields
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	if(pointNormalMode == 0){
		pclUtilitys::getPointNormalsFromCloudRgb(*input, *cloud_with_normals);
	}else{
		pclUtilitys::getSmoothPointNormalsFromCloudRgb(*input, *cloud_with_normals);
	}

	// Create search tree
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);	
	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.025);//0.025
	// Set typical values for the parameters
	gp3.setMu (2.5);//2.5
	gp3.setMaximumNearestNeighbors (50);//100
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees = M_PI/18
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);	


	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	// Additional vertex information
	//std::vector<int> parts = gp3.getPartIDs();
	//std::vector<int> states = gp3.getPointStates();		
}

/**
  * meshing via Fast Triangulation - greedy projection
  * input: const pcl::PointCloud<pcl::PointXYZRGB>::Ptr  \n
  * \param	filepath
  * \return	void 
  *
  */
void pclUtilitys::meshingViaFastTriangulation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, std::string filePath, int pointNormalMode){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pclUtilitys::convertXYZRGBtoXYZ(*input, cloud);

	// Concatenate the XYZ and normal fields
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	if(pointNormalMode == 0){
		pclUtilitys::getPointNormalsFromCloudRgb(*input, *cloud_with_normals);
	}else{
		pclUtilitys::getSmoothPointNormalsFromCloudRgb(*input, *cloud_with_normals);
	}

	//pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);	
	// Create search tree
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);	
	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.025);
	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);	

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();	

	//save the result
	//pcl::io::saveOBJFile("/var/www/ramdev/meshFast.obj", triangles);
	pcl::io::savePolygonFileSTL(filePath, triangles);
}

/**
  * meshing via B-Spline Fitting
  * input: const pcl::PointCloud<pcl::PointXYZRGB>::Ptr \n
  * output: cl::PolygonMesh &triangles \n
  * \param	
  * \return	void 
  *
  */
 //reference: https://pcl.readthedocs.io/projects/tutorials/en/latest/bspline_fitting.html#bspline-fitting
void pclUtilitys::meshingBsplineFitting(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, pcl::PolygonMesh &mesh){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pclUtilitys::convertXYZRGBtoXYZ(*input, cloud);
	pcl::on_nurbs::NurbsDataSurface data;
	PointCloud2Vector3d (cloud, data.interior);

	// ############################################################################
	// fit B-spline surface
	// parameters
	unsigned order (3);
	unsigned refinement (5); //5
	unsigned iterations (10); //10
	unsigned mesh_resolution (256);//256

	pcl::on_nurbs::FittingSurface::Parameter params;
	params.interior_smoothness = 0.2;
	params.interior_weight = 1.0;
	params.boundary_smoothness = 0.2;
	params.boundary_weight = 0.0;

	// initialize
	ON_NurbsSurface nurbs = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox (order, &data);
	pcl::on_nurbs::FittingSurface fit (&data, nurbs);
	//  fit.setQuiet (false); // enable/disable debug output
	// mesh for visualization
	pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::Vertices> mesh_vertices;
	std::string mesh_id = "mesh_nurbs";
	pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh (fit.m_nurbs, mesh, mesh_resolution);

	// surface refinement
	unsigned int i = 0;
	unsigned int imax = refinement;
	while(i<imax){
		fit.refine (0);
		fit.refine (1);
		fit.assemble (params);
		fit.solve ();
		pcl::on_nurbs::Triangulation::convertSurface2Vertices (fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
	i++;
	}

	// surface fitting with final refinement level
	unsigned int j = 0;
	unsigned int jmax = iterations;
	while(j<jmax){
		fit.assemble (params);
		fit.solve ();
		pcl::on_nurbs::Triangulation::convertSurface2Vertices (fit.m_nurbs, mesh_cloud, mesh_vertices, mesh_resolution);
	j++;
	}

	// ############################################################################
	// fit B-spline curve
	// parameters
	pcl::on_nurbs::FittingCurve2dAPDM::FitParameter curve_params;
	curve_params.addCPsAccuracy = 5e-2;
	curve_params.addCPsIteration = 3;
	curve_params.maxCPs = 200;
	curve_params.accuracy = 1e-3;
	curve_params.iterations = 100;
	curve_params.param.closest_point_resolution = 0;
	curve_params.param.closest_point_weight = 1.0;
	curve_params.param.closest_point_sigma2 = 0.1;
	curve_params.param.interior_sigma2 = 0.00001;
	curve_params.param.smooth_concavity = 1.0;
	curve_params.param.smoothness = 1.0;

	// initialisation (circular)
	pcl::on_nurbs::NurbsDataCurve2d curve_data;
	curve_data.interior = data.interior_param;
	curve_data.interior_weight_function.push_back (true);
	ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D (order, curve_data.interior);

	// curve fitting
	pcl::on_nurbs::FittingCurve2dASDM curve_fit (&curve_data, curve_nurbs);

	// curve_fit.setQuiet (false); // enable/disable debug output
	curve_fit.fitting (curve_params);

	// ############################################################################
	// triangulation of trimmed surface
	pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh (fit.m_nurbs, curve_fit.m_nurbs, mesh, mesh_resolution);
}

void pclUtilitys::PointCloud2Vector3d (pcl::PointCloud<PointXYZ>::Ptr cloud, pcl::on_nurbs::vector_vec3d &data)
{
	unsigned int i = 0;
	unsigned int imax = cloud->size();
	while(i < imax){		
		pcl::PointXYZ &p = cloud->at(i);
		if (!std::isnan (p.x) && !std::isnan (p.y) && !std::isnan (p.z)){
			data.push_back (Eigen::Vector3d (p.x, p.y, p.z));
		}
	i++;
	}
}
#endif