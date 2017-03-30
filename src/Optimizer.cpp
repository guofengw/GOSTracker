#include "stdafx.h"
#include "Optimizer.h"  //


#include <fstream>      //C++
#include <iostream>

#include <opencv2\highgui\highgui.hpp>  //other project
#include <opencv2\calib3d\calib3d.hpp>

#include "levmar.h"  //Current project
#include "Param.h"
extern ols::CameraCalibration g_calibration;
using namespace ols;


inline float weight_tukey(float r, float c)
{
	
	if(fabs(r)<0.0000001)
		return 1.0f;
	float value = 0.0f;
	if(std::fabs(r) <= c)
	{
		value = (c*c/6)*(1-std::pow((1-r*r/(c*c)),3));
		
		//value = r*(1-r*r/(c*c))*(1-r*r/(c*c));
		
	}
	else
		value = c*c/6;
		//value = 0;
	
	
	value = std::sqrt(value)/r;
	return fabs(value);

}

inline float tukey(float r, float c)
{
	float value = 0.0f;
	if(std::fabs(r) <= c)
		value = (c*c/6)*(1-std::pow((1-r*r/(c*c)),3));
	else
		value = c*c/6;
	//value = r*r;
	return value;
}

inline float tukey_derivation(float r, float c)
{
	float value = 0.0f;
	if(std::fabs(r) <= c)
		value = 0.5*std::pow(1-r/(c*c),2.0f);
	else
		value = 0;
	return value;
}

inline void num2string(int num, string &s)
{
	s.clear();
	string c;
	int temp;
	temp = num/10;
	while(temp>0)
	{
		c = num - temp*10 + 48;
		s = c + s;
		num = temp;
		temp = num/10;

	}
	c = num + 48;
	s = c + s;
	while(s.length() < 5)
	{
		s = "0" + s;
	}

}



Optimizer::Optimizer(const Config& config, const cv::Mat& initPose, bool is_writer) 
{
	m_data.m_model = new Model(config);
	//m_data.m_model->LoadGLMModel(config.filename.c_str());
	m_data.m_correspondence = new Correspondence(config.width,config.height);
	m_data.m_correspondence->m_lineBundle.create(R,2*L+1,CV_8UC3);
	m_data.m_correspondence->m_hsv.create(R,2*L+1,CV_8UC3);
	m_calibration = config.camCalibration;
	m_data.m_model->InitPose(initPose);

	m_is_writer = is_writer;
	if(is_writer)
	{
		//record the pose
		std::string poseFile = "data/gos_pose.txt";
		m_outPose.open(poseFile);
		if( !m_outPose.is_open() )
		{
			printf("Cannot write the pose\n");
			return;
		}
	}
}

Optimizer::~Optimizer()
{
	if(m_is_writer)
	{
		m_outPose.close();
	}
}

void Optimizer::computeExtrinsic(const cv::Mat& prepose, cv::Mat& extrinsic)
{
	float p[6] = {0};
	p[0] = prepose.at<float>(0,0); p[1] = prepose.at<float>(0,1); p[2] = prepose.at<float>(0,2);
	p[3] = prepose.at<float>(0,3); p[4] = prepose.at<float>(0,4); p[5] = prepose.at<float>(0,5);
	m_data.m_model->computeExtrinsicByEuler(&extrinsic,p[0],p[1],p[2],p[3],p[4],p[5]);
}



/********************************************Begin:以下为用LM求解非线性最小二乘******************************************************/

cv::Mat Optimizer::optimizingLM(cv::Mat& prepose, cv::Mat& frame, int frameId)
{
	cv::Mat raux(3,1,CV_64FC1);
	cv::Mat taux(3,1,CV_64FC1);
	

	int m = 6;

	double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
	opts[0]=1; opts[0] = 1E-1; opts[1]=1E-15; opts[2]=1E-15; opts[3]=1E-20;
	opts[4]= LM_DIFF_DELTA; //

	double p[6] = {0};
	p[0] = prepose.at<float>(0,0); p[1] = prepose.at<float>(0,1); p[2] = prepose.at<float>(0,2);
	p[3] = prepose.at<float>(0,3); p[4] = prepose.at<float>(0,4); p[5] = prepose.at<float>(0,5);
	double pre[6] = {0};
	pre[0]= prepose.at<float>(0,0); pre[1] = prepose.at<float>(0,1); pre[2] = prepose.at<float>(0,2);
	pre[3] = prepose.at<float>(0,3); pre[4] = prepose.at<float>(0,4); pre[5] = prepose.at<float>(0,5);
	int64 time0 = cv::getTickCount();
	int n = R;
	//double x[2*R];

	for(int index = 0; index<5; index++)  //固定迭代5次
	{

		m_data.m_pointset = m_data.m_model->GetVisibleModelPointsCV(prepose,R);

		n = m_data.m_pointset.m_img_points.size();
		double *x = new double[2*n];
		for(int i=0; i<2*n; i++)
			x[i] = 0.0;

		m_data.m_correspondence->FindCorrespondingPointsByDP(frame,m_data.m_pointset);  //5ms
		//m_data.m_correspondence->FindCorrespondingPointsInImage(frame,m_data.m_pointset);  
		
		
#ifdef MY
		int ret = dlevmar_der(lm,jaclm,p,x,m,2*n,1000,opts,info,NULL,NULL,&m_data);
		prepose.at<float>(0,0)=p[0]; prepose.at<float>(0,1)=p[1]; prepose.at<float>(0,2)=p[2]; 
		prepose.at<float>(0,3)=p[3]; prepose.at<float>(0,4)=p[4]; prepose.at<float>(0,5)=p[5];
#else
		taux.at<double>(0,0) = prepose.at<float>(0,0); taux.at<double>(1,0) = prepose.at<float>(0,1); taux.at<double>(2,0) = prepose.at<float>(0,2);
		raux.at<double>(0,0) = prepose.at<float>(0,3); raux.at<double>(1,0) = prepose.at<float>(0,4); raux.at<double>(2,0) = prepose.at<float>(0,5);
		INT64 time_s = cv::getTickCount();
		if(m_data.m_pointset.m_model_points.size() > 0)
			cv::solvePnP(m_data.m_pointset.m_model_points,m_data.m_pointset.m_correspoinding_points,m_calibration.getIntrinsic(),m_calibration.getDistorsion(),raux,taux,true);
		
		INT64 time_s1 = cv::getTickCount();
		//printf("solving time:%lf\n",(time_s1-time_s)/cv::getTickFrequency());
		raux.convertTo(m_data.m_model->m_rvec,CV_32F);
		taux.convertTo(m_data.m_model->m_tvec,CV_32F);
		prepose.at<float>(0,0) = m_data.m_model->m_tvec.at<float>(0,0); prepose.at<float>(0,1) = m_data.m_model->m_tvec.at<float>(1,0); prepose.at<float>(0,2) = m_data.m_model->m_tvec.at<float>(2,0);
		prepose.at<float>(0,3) = m_data.m_model->m_rvec.at<float>(0,0); prepose.at<float>(0,4) = m_data.m_model->m_rvec.at<float>(1,0); prepose.at<float>(0,5) = m_data.m_model->m_rvec.at<float>(2,0);
#endif
		

		delete []x;	

	}

	//update histogram
	m_data.m_correspondence->updateHist();


	//OpenGL显示
	m_data.m_model->DisplayGL(prepose);
	//OpenCV显示
	m_data.m_model->DisplayCV(prepose,frame);
	


	//display the frame id
	char num[100];
	itoa(frameId,num,10);
	std::string text(num);
	cv::putText(frame,text,cv::Point(10,30),FONT_HERSHEY_COMPLEX,1,cv::Scalar(0,0,255),2);
	cv::imshow("lines",m_data.m_correspondence->m_lineBundle);
	cv::imshow("curFrame",frame);

	if(m_is_writer)
	{
		m_outPose << frameId << " " << prepose.at<float>(0,3) << " " << prepose.at<float>(0,4) << " " << prepose.at<float>(0,5) << " "
			<< prepose.at<float>(0,0) << " "
			<< prepose.at<float>(0,1) << " "
			<< prepose.at<float>(0,2) << std::endl;
	}


	return prepose.clone();

}


void Optimizer::lm(double *p, double *x, int m, int n, void* data)
{
	
	Model *model = ((Data*)data)->m_model;
	Correspondence *corres = ((Data*)data)->m_correspondence;	

	cv::Mat pose(1,6,CV_32FC1);
	pose.at<float>(0,0) = p[0]; pose.at<float>(0,1) = p[1]; pose.at<float>(0,2) = p[2];
	pose.at<float>(0,3) = p[3]; pose.at<float>(0,4) = p[4]; pose.at<float>(0,5) = p[5];

	int64 time0 = cv::getTickCount();
	PointSet& pointset = ((Data*)data)->m_pointset;
	model->GetImagePoints(pose,pointset);

	float c = 15;

	//printf(".............................c:%lf\n",c);

	float r1, r2;
	Data* m_data = ((Data*)data);
	for(int i=0, j=0; i<n/2; i++)
	{
		r1 = pointset.m_img_points_f[i].x - pointset.m_correspoinding_points[i].x;
		r2 = pointset.m_img_points_f[i].y - pointset.m_correspoinding_points[i].y;
		//cout<<r1<<" "<<r2<<endl;

		x[j++] = weight_tukey(r1,c)*r1;
		x[j++] = weight_tukey(r2,c)*r2;

		
	}

	int64 time1 = cv::getTickCount();
	//printf("lm time:%lfsec\n",(time1-time0)/cv::getTickFrequency());

}

inline float durx(const cv::Mat& pose, const cv::Mat& pos)
{
	float pi = 3.1415926;
	float t1 = pose.at<float>(0,0); float t2 = pose.at<float>(0,1); float t3 = pose.at<float>(0,2);
	float rx = pose.at<float>(0,3); float ry = pose.at<float>(0,4); float rz = pose.at<float>(0,5);
	float fx = g_calibration.fx(); float fy = g_calibration.fy(); float cx = g_calibration.cx(); float cy = g_calibration.cy();

	float X = pos.at<float>(0,0); float Y = pos.at<float>(1,0); float Z = pos.at<float>(2,0);

	//float result = -((Y*cos(rx)*cos(ry) - Z*cos(ry)*sin(rx))*(Z*(fx*(sin(rx)*sin(rz) + cos(rx)*cos(rz)*sin(ry)) + cx*cos(rx)*cos(ry)) + cx*t3 + fx*t1 - Y*(fx*(cos(rx)*sin(rz) - cos(rz)*sin(rx)*sin(ry)) - cx*cos(ry)*sin(rx)) - X*(cx*sin(ry) - fx*cos(ry)*cos(rz))) - (Y*(fx*(sin(rx)*sin(rz) + cos(rx)*cos(rz)*sin(ry)) + cx*cos(rx)*cos(ry)) + Z*(fx*(cos(rx)*sin(rz) - cos(rz)*sin(rx)*sin(ry)) - cx*cos(ry)*sin(rx)))*(t3 - X*sin(ry) + Z*cos(rx)*cos(ry) + Y*cos(ry)*sin(rx)))/std::pow((t3 - X*sin(ry) + Z*cos(rx)*cos(ry) + Y*cos(ry)*sin(rx)),2.0f);
	float result = -((Y*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - Z*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)))*(cx*t3 + fx*t1 + X*(fx*cos(ry)*cos(rz) - cx*cos(rz)*sin(ry)) + Y*(cx*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + fx*(sin(rx)*sin(ry) - cos(rx)*cos(ry)*sin(rz))) + Z*(cx*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) + fx*(cos(rx)*sin(ry) + cos(ry)*sin(rx)*sin(rz)))) - (Y*(cx*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) + fx*(cos(rx)*sin(ry) + cos(ry)*sin(rx)*sin(rz))) - Z*(cx*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + fx*(sin(rx)*sin(ry) - cos(rx)*cos(ry)*sin(rz))))*(t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry)))/std::pow((t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry)),2.0f);
	return result;
}

inline float dury(const cv::Mat& pose, const cv::Mat& pos)
{
	float pi = 3.1415926;
	float t1 = pose.at<float>(0,0); float t2 = pose.at<float>(0,1); float t3 = pose.at<float>(0,2);
	float rx = pose.at<float>(0,3); float ry = pose.at<float>(0,4); float rz = pose.at<float>(0,5);
	float fx = g_calibration.fx(); float fy = g_calibration.fy(); float cx = g_calibration.cx(); float cy = g_calibration.cy();
	
	float X = pos.at<float>(0,0); float Y = pos.at<float>(1,0); float Z = pos.at<float>(2,0);

	//float result = ((X*cos(ry) + Z*cos(rx)*sin(ry) + Y*sin(rx)*sin(ry))*(Z*(fx*(sin(rx)*sin(rz) + cos(rx)*cos(rz)*sin(ry)) + cx*cos(rx)*cos(ry)) + cx*t3 + fx*t1 - Y*(fx*(cos(rx)*sin(rz) - cos(rz)*sin(rx)*sin(ry)) - cx*cos(ry)*sin(rx)) - X*(cx*sin(ry) - fx*cos(ry)*cos(rz))) - (Z*(cx*cos(rx)*sin(ry) - fx*cos(rx)*cos(ry)*cos(rz)) + Y*(cx*sin(rx)*sin(ry) - fx*cos(ry)*cos(rz)*sin(rx)) + X*(cx*cos(ry) + fx*cos(rz)*sin(ry)))*(t3 - X*sin(ry) + Z*cos(rx)*cos(ry) + Y*cos(ry)*sin(rx)))/std::pow((t3 - X*sin(ry) + Z*cos(rx)*cos(ry) + Y*cos(ry)*sin(rx)),2.0f);
	float result = ((Y*(sin(rx)*sin(ry) - cos(rx)*cos(ry)*sin(rz)) + Z*(cos(rx)*sin(ry) + cos(ry)*sin(rx)*sin(rz)) + X*cos(ry)*cos(rz))*(cx*t3 + fx*t1 + X*(fx*cos(ry)*cos(rz) - cx*cos(rz)*sin(ry)) + Y*(cx*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + fx*(sin(rx)*sin(ry) - cos(rx)*cos(ry)*sin(rz))) + Z*(cx*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) + fx*(cos(rx)*sin(ry) + cos(ry)*sin(rx)*sin(rz)))) - (X*(cx*cos(ry)*cos(rz) + fx*cos(rz)*sin(ry)) + Y*(cx*(sin(rx)*sin(ry) - cos(rx)*cos(ry)*sin(rz)) - fx*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz))) + Z*(cx*(cos(rx)*sin(ry) + cos(ry)*sin(rx)*sin(rz)) - fx*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz))))*(t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry)))/std::pow((t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry)),2.0f);
	return result;

}

inline float durz(const cv::Mat& pose, const cv::Mat& pos)
{
	float pi = 3.1415926;
	float t1 = pose.at<float>(0,0); float t2 = pose.at<float>(0,1); float t3 = pose.at<float>(0,2);
	float rx = pose.at<float>(0,3); float ry = pose.at<float>(0,4); float rz = pose.at<float>(0,5);
	float fx = g_calibration.fx(); float fy = g_calibration.fy(); float cx = g_calibration.cx(); float cy = g_calibration.cy();
	
	float X = pos.at<float>(0,0); float Y = pos.at<float>(1,0); float Z = pos.at<float>(2,0);
	
	//float result = -(fx*Y*(cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz)) - fx*Z*(cos(rz)*sin(rx) - cos(rx)*sin(ry)*sin(rz)) + fx*X*cos(ry)*sin(rz))/(t3 - X*sin(ry) + Z*cos(rx)*cos(ry) + Y*cos(ry)*sin(rx));
	float result = -((X*sin(ry)*sin(rz) + Y*cos(rx)*cos(rz)*sin(ry) - Z*cos(rz)*sin(rx)*sin(ry))*(cx*t3 + fx*t1 + X*(fx*cos(ry)*cos(rz) - cx*cos(rz)*sin(ry)) + Y*(cx*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + fx*(sin(rx)*sin(ry) - cos(rx)*cos(ry)*sin(rz))) + Z*(cx*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) + fx*(cos(rx)*sin(ry) + cos(ry)*sin(rx)*sin(rz)))) + (Y*(fx*cos(rx)*cos(ry)*cos(rz) - cx*cos(rx)*cos(rz)*sin(ry)) - Z*(fx*cos(ry)*cos(rz)*sin(rx) - cx*cos(rz)*sin(rx)*sin(ry)) + X*(fx*cos(ry)*sin(rz) - cx*sin(ry)*sin(rz)))*(t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry)))/std::pow((t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry)),2.0f);
	return result;
}

inline float dux(const cv::Mat& pose, const cv::Mat& pos)
{
	float pi = 3.1415926;
	float t1 = pose.at<float>(0,0); float t2 = pose.at<float>(0,1); float t3 = pose.at<float>(0,2);
	float rx = pose.at<float>(0,3); float ry = pose.at<float>(0,4); float rz = pose.at<float>(0,5);
	float fx = g_calibration.fx(); float fy = g_calibration.fy(); float cx = g_calibration.cx(); float cy = g_calibration.cy();
	
	float X = pos.at<float>(0,0); float Y = pos.at<float>(1,0); float Z = pos.at<float>(2,0);

	//float result = fx/(t3 - X*sin(ry) + Z*cos(rx)*cos(ry) + Y*cos(ry)*sin(rx));
	float result = fx/(t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry));
	return result;
}

inline float duy(const cv::Mat& pose, const cv::Mat& pos)
{
	
	float result = 0;
	return result;
}

inline float duz(const cv::Mat& pose, const cv::Mat& pos)
{
	float pi = 3.1415926;
	float t1 = pose.at<float>(0,0); float t2 = pose.at<float>(0,1); float t3 = pose.at<float>(0,2);
	float rx = pose.at<float>(0,3); float ry = pose.at<float>(0,4); float rz = pose.at<float>(0,5);
	float fx = g_calibration.fx(); float fy = g_calibration.fy(); float cx = g_calibration.cx(); float cy = g_calibration.cy();
	
	float X = pos.at<float>(0,0); float Y = pos.at<float>(1,0); float Z = pos.at<float>(2,0);

	//float result = -(Z*(fx*(sin(rx)*sin(rz) + cos(rx)*cos(rz)*sin(ry)) + cx*cos(rx)*cos(ry)) + cx*t3 - cx*(t3 - X*sin(ry) + Z*cos(rx)*cos(ry) + Y*cos(ry)*sin(rx)) + fx*t1 - Y*(fx*(cos(rx)*sin(rz) - cos(rz)*sin(rx)*sin(ry)) - cx*cos(ry)*sin(rx)) - X*(cx*sin(ry) - fx*cos(ry)*cos(rz)))/std::pow((t3 - X*sin(ry) + Z*cos(rx)*cos(ry) + Y*cos(ry)*sin(rx)),2.0f);
	float result = -(cx*t3 + fx*t1 + X*(fx*cos(ry)*cos(rz) - cx*cos(rz)*sin(ry)) - cx*(t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry)) + Y*(cx*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + fx*(sin(rx)*sin(ry) - cos(rx)*cos(ry)*sin(rz))) + Z*(cx*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) + fx*(cos(rx)*sin(ry) + cos(ry)*sin(rx)*sin(rz))))/std::pow((t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry)),2.0f);
	return result;
}

inline float dvrx(const cv::Mat& pose, const cv::Mat& pos)
{
	float pi = 3.1415926;
	float t1 = pose.at<float>(0,0); float t2 = pose.at<float>(0,1); float t3 = pose.at<float>(0,2);
	float rx = pose.at<float>(0,3); float ry = pose.at<float>(0,4); float rz = pose.at<float>(0,5);
	float fx = g_calibration.fx(); float fy = g_calibration.fy(); float cx = g_calibration.cx(); float cy = g_calibration.cy();
	
	float X = pos.at<float>(0,0); float Y = pos.at<float>(1,0); float Z = pos.at<float>(2,0);

	//float result = -((Y*cos(rx)*cos(ry) - Z*cos(ry)*sin(rx))*(cy*t3 - Z*(fy*(cos(rz)*sin(rx) - cos(rx)*sin(ry)*sin(rz)) - cy*cos(rx)*cos(ry)) + fy*t2 + Y*(fy*(cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz)) + cy*cos(ry)*sin(rx)) - X*(cy*sin(ry) - fy*cos(ry)*sin(rz))) + (Y*(fy*(cos(rz)*sin(rx) - cos(rx)*sin(ry)*sin(rz)) - cy*cos(rx)*cos(ry)) + Z*(fy*(cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz)) + cy*cos(ry)*sin(rx)))*(t3 - X*sin(ry) + Z*cos(rx)*cos(ry) + Y*cos(ry)*sin(rx)))/std::pow((t3 - X*sin(ry) + Z*cos(rx)*cos(ry) + Y*cos(ry)*sin(rx)),2.0f);
	float result = -((Z*(cy*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + fy*cos(rx)*cos(rz)) - Y*(cy*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - fy*cos(rz)*sin(rx)))*(t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry)) + (Y*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - Z*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)))*(Y*(cy*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + fy*cos(rx)*cos(rz)) + cy*t3 + fy*t2 + Z*(cy*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - fy*cos(rz)*sin(rx)) + X*(fy*sin(rz) - cy*cos(rz)*sin(ry))))/std::pow((t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry)),2.0f);
	return result;
}

inline float dvry(const cv::Mat& pose, const cv::Mat& pos)
{
	float pi = 3.1415926;
	float t1 = pose.at<float>(0,0); float t2 = pose.at<float>(0,1); float t3 = pose.at<float>(0,2);
	float rx = pose.at<float>(0,3); float ry = pose.at<float>(0,4); float rz = pose.at<float>(0,5);
	float fx = g_calibration.fx(); float fy = g_calibration.fy(); float cx = g_calibration.cx(); float cy = g_calibration.cy();
	
	float X = pos.at<float>(0,0); float Y = pos.at<float>(1,0); float Z = pos.at<float>(2,0);

	
	//float result = ((X*cos(ry) + Z*cos(rx)*sin(ry) + Y*sin(rx)*sin(ry))*(cy*t3 - Z*(fy*(cos(rz)*sin(rx) - cos(rx)*sin(ry)*sin(rz)) - cy*cos(rx)*cos(ry)) + fy*t2 + Y*(fy*(cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz)) + cy*cos(ry)*sin(rx)) - X*(cy*sin(ry) - fy*cos(ry)*sin(rz))) - (Z*(cy*cos(rx)*sin(ry) - fy*cos(rx)*cos(ry)*sin(rz)) + Y*(cy*sin(rx)*sin(ry) - fy*cos(ry)*sin(rx)*sin(rz)) + X*(cy*cos(ry) + fy*sin(ry)*sin(rz)))*(t3 - X*sin(ry) + Z*cos(rx)*cos(ry) + Y*cos(ry)*sin(rx)))/std::pow((t3 - X*sin(ry) + Z*cos(rx)*cos(ry) + Y*cos(ry)*sin(rx)),2.0f);
	float result = ((Y*(sin(rx)*sin(ry) - cos(rx)*cos(ry)*sin(rz)) + Z*(cos(rx)*sin(ry) + cos(ry)*sin(rx)*sin(rz)) + X*cos(ry)*cos(rz))*(Y*(cy*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + fy*cos(rx)*cos(rz)) + cy*t3 + fy*t2 + Z*(cy*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - fy*cos(rz)*sin(rx)) + X*(fy*sin(rz) - cy*cos(rz)*sin(ry))) - (cy*Y*(sin(rx)*sin(ry) - cos(rx)*cos(ry)*sin(rz)) + cy*Z*(cos(rx)*sin(ry) + cos(ry)*sin(rx)*sin(rz)) + cy*X*cos(ry)*cos(rz))*(t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry)))/std::pow((t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry)),2.0f);
	return result;
}

inline float dvrz(const cv::Mat& pose, const cv::Mat& pos)
{
	float pi = 3.1415926;
	float t1 = pose.at<float>(0,0); float t2 = pose.at<float>(0,1); float t3 = pose.at<float>(0,2);
	float rx = pose.at<float>(0,3); float ry = pose.at<float>(0,4); float rz = pose.at<float>(0,5);
	float fx = g_calibration.fx(); float fy = g_calibration.fy(); float cx = g_calibration.cx(); float cy = g_calibration.cy();
	
	float X = pos.at<float>(0,0); float Y = pos.at<float>(1,0); float Z = pos.at<float>(2,0);

	//float result = (fy*Z*(sin(rx)*sin(rz) + cos(rx)*cos(rz)*sin(ry)) - fy*Y*(cos(rx)*sin(rz) - cos(rz)*sin(rx)*sin(ry)) + fy*X*cos(ry)*cos(rz))/(t3 - X*sin(ry) + Z*cos(rx)*cos(ry) + Y*cos(ry)*sin(rx));
	float result = ((Z*(fy*sin(rx)*sin(rz) - cy*cos(rz)*sin(rx)*sin(ry)) - Y*(fy*cos(rx)*sin(rz) - cy*cos(rx)*cos(rz)*sin(ry)) + X*(fy*cos(rz) + cy*sin(ry)*sin(rz)))*(t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry)) - (X*sin(ry)*sin(rz) + Y*cos(rx)*cos(rz)*sin(ry) - Z*cos(rz)*sin(rx)*sin(ry))*(Y*(cy*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + fy*cos(rx)*cos(rz)) + cy*t3 + fy*t2 + Z*(cy*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - fy*cos(rz)*sin(rx)) + X*(fy*sin(rz) - cy*cos(rz)*sin(ry))))/std::pow((t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry)),2.0f);
	return result;
}

inline float dvx(const cv::Mat& pose, const cv::Mat& pos)
{

	return 0;
}

inline float dvy(const cv::Mat& pose, const cv::Mat& pos)
{
	float pi = 3.1415926;
	float t1 = pose.at<float>(0,0); float t2 = pose.at<float>(0,1); float t3 = pose.at<float>(0,2);
	float rx = pose.at<float>(0,3); float ry = pose.at<float>(0,4); float rz = pose.at<float>(0,5);
	float fx = g_calibration.fx(); float fy = g_calibration.fy(); float cx = g_calibration.cx(); float cy = g_calibration.cy();
	
	float X = pos.at<float>(0,0); float Y = pos.at<float>(1,0); float Z = pos.at<float>(2,0);

	//float result = fy/(t3 - X*sin(ry) + Z*cos(rx)*cos(ry) + Y*cos(ry)*sin(rx));
	float result = fy/(t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry));
	return result;
}

inline float dvz(const cv::Mat& pose, const cv::Mat& pos)
{
	float pi = 3.1415926;
	float t1 = pose.at<float>(0,0); float t2 = pose.at<float>(0,1); float t3 = pose.at<float>(0,2);
	float rx = pose.at<float>(0,3); float ry = pose.at<float>(0,4); float rz = pose.at<float>(0,5);
	float fx = g_calibration.fx(); float fy = g_calibration.fy(); float cx = g_calibration.cx(); float cy = g_calibration.cy();
	
	float X = pos.at<float>(0,0); float Y = pos.at<float>(1,0); float Z = pos.at<float>(2,0);

	//float result = (Z*(fy*(cos(rz)*sin(rx) - cos(rx)*sin(ry)*sin(rz)) - cy*cos(rx)*cos(ry)) - cy*t3 + cy*(t3 - X*sin(ry) + Z*cos(rx)*cos(ry) + Y*cos(ry)*sin(rx)) - fy*t2 - Y*(fy*(cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz)) + cy*cos(ry)*sin(rx)) + X*(cy*sin(ry) - fy*cos(ry)*sin(rz)))/std::pow((t3 - X*sin(ry) + Z*cos(rx)*cos(ry) + Y*cos(ry)*sin(rx)),2.0f);
	float result = -(Y*(cy*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + fy*cos(rx)*cos(rz)) + cy*t3 + fy*t2 + Z*(cy*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - fy*cos(rz)*sin(rx)) - cy*(t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry)) + X*(fy*sin(rz) - cy*cos(rz)*sin(ry)))/std::pow((t3 + Y*(cos(ry)*sin(rx) + cos(rx)*sin(ry)*sin(rz)) + Z*(cos(rx)*cos(ry) - sin(rx)*sin(ry)*sin(rz)) - X*cos(rz)*sin(ry)),2.0f);
	return result;

}

void Optimizer::jaclm(double *p, double *jac, int m, int n, void* data)
{
	
	Model *model = ((Data*)data)->m_model;
	Correspondence *corres = ((Data*)data)->m_correspondence;
	PointSet& pointset = ((Data*)data)->m_pointset;

	static cv::Mat pose(1,6,CV_32FC1);
	pose.at<float>(0,0) = p[0]; pose.at<float>(0,1) = p[1]; pose.at<float>(0,2) = p[2];
	pose.at<float>(0,3) = p[3]; pose.at<float>(0,4) = p[4]; pose.at<float>(0,5) = p[5];
	static cv::Mat pos(3,1,CV_32FC1);

	float c = 15;

	Data* m_data = ((Data*)data);
	float r1,r2,w1,w2;
	int64 time0 = cv::getTickCount();
	for(int i=0,j=0; i<n/2; i++)
	{
		r1 = pointset.m_img_points_f[i].x - pointset.m_correspoinding_points[i].x;
		r2 = pointset.m_img_points_f[i].y - pointset.m_correspoinding_points[i].y;

		w1 = weight_tukey(r1,c);  //假的robust estimate处理方式，没有真正实现！！！
		w2 = weight_tukey(r2,c);
		
		pos.at<float>(0,0) = pointset.m_model_points[i].x;
		pos.at<float>(1,0) = pointset.m_model_points[i].y;
		pos.at<float>(2,0) = pointset.m_model_points[i].z;


		jac[j++] = w1*dux(pose,pos);
		jac[j++] = w1*duy(pose,pos);
		jac[j++] = w1*duz(pose,pos);
		jac[j++] = w1*durx(pose,pos);
		jac[j++] = w1*dury(pose,pos);
		jac[j++] = w1*durz(pose,pos);

		jac[j++] = w2*dvx(pose,pos);
		jac[j++] = w2*dvy(pose,pos);
		jac[j++] = w2*dvz(pose,pos);
		jac[j++] = w2*dvrx(pose,pos);
		jac[j++] = w2*dvry(pose,pos);
		jac[j++] = w2*dvrz(pose,pos);
	}
	int64 time1 = cv::getTickCount();
	//printf("jac time:%lf\n",(time1-time0)/cv::getTickFrequency());

}

/**************************************End:以上为用LM求解非线性最小二乘**********************************************/