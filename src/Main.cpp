// RegistrationByLS.cpp : 定义控制台应用程序的入口点。
//

//#include "stdafx.h"
#include "Render.h"
#include "Optimizer.h"
#include "Config.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <opencv2\highgui\highgui.hpp>



osa::Render g_render;
ols::CameraCalibration g_calibration;


int main(int argc, char* argv[])
{
	float distorsion[5] = {-4.0817887112457034e-002, 4.2918200942843993e-001, 0., 0.,
		-1.1326434091563919e+000};
	ols::CameraCalibration calibration(727.55,727.55,479.5,269.5,distorsion); //479.5,269.5
	g_calibration = calibration;

	cv::VideoCapture capture;
	
	//输入视频数据
	if(!(capture.open("bunny.avi")))
	{
		std::cout<<"can not find video!"<<std::endl;
		return 0;
	}
	/*if(!capture.open(0))
	{
		std::cout<<"No camera!"<<std::endl;
		return 0;
	}*/

	ols::Config config;
	config.camCalibration = calibration;
	config.width = (int)capture.get(CV_CAP_PROP_FRAME_WIDTH);
	config.height = (int)capture.get(CV_CAP_PROP_FRAME_HEIGHT);
	config.filename = "bunny.obj";
	config.model = glmReadOBJ(const_cast<char*>(config.filename.c_str()));

	//打开opengl
	g_render.init(calibration,config.width,config.height,argc,argv);
	

	cv::Mat pose(1,6,CV_32FC1);

	//初始姿态，需要预先给定
	//bunny
	pose.at<float>(0,0)=23.018f; pose.at<float>(0,1)=-24.497f; pose.at<float>(0,2)=412.035f;
	pose.at<float>(0,3)=2.268f; pose.at<float>(0,4)=0.486f; pose.at<float>(0,5)=0.385;

	//cube
	/*pose.at<float>(0,0)=5.33f; pose.at<float>(0,1)=1.16; pose.at<float>(0,2)=25.95f;
	pose.at<float>(0,3)=2.11f; pose.at<float>(0,4)=0.28f; pose.at<float>(0,5)=0.08;*/



	bool is_writer = false;
	ols::Optimizer optimizer(config,pose,is_writer);  

	int frameId = 0;
	int key = 0;

	cv::Mat curFrame;
	while(capture.read(curFrame))
	{

		frameId++;

		int64 time0 = cv::getTickCount();	
	
		pose = optimizer.optimizingLM(pose,curFrame,frameId);
			
		//printf("pose:%f,%f,%f,%f,%f,%f\n",pose.at<float>(0,0),pose.at<float>(0,1),pose.at<float>(0,2),pose.at<float>(0,3),pose.at<float>(0,4),pose.at<float>(0,5));

		int64 time1 = cv::getTickCount();
		printf("fps:%f\n",1.0f/((time1-time0)/cv::getTickFrequency()));
				
		key = cv::waitKey(1);
		if(key == 'q' || key == 'Q')
			break;		

	}

	return 0;
}

