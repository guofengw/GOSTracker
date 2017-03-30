#include "stdafx.h"
#include "CameraCalibration.h"

using namespace ols;

CameraCalibration::CameraCalibration() : m_intrinsic(3,3,CV_32FC1), m_extrinsic(4,4,CV_32FC1)
{
	m_intrinsic1 = cv::Matx33f::eye();
	m_distortion1 = cv::Matx<float,5,1>::zeros();

}

CameraCalibration::CameraCalibration(float _fx, float _fy, float _cx, float _cy) : m_intrinsic(3,3,CV_32FC1), m_extrinsic(4,4,CV_32FC1)
{


    fx() = _fx;
    fy() = _fy;
    cx() = _cx;
    cy() = _cy;

	m_distortion.resize(5);
	for(int i=0; i<5; i++)
		m_distortion[i] = 0.0f;

	//
	m_intrinsic1 = cv::Matx33f::eye();
	m_distortion1 = cv::Matx<float,5,1>::zeros();
	m_intrinsic1(0,0) = _fx;
	m_intrinsic1(1,1) = _fy;
	m_intrinsic1(0,2) = _cx;
	m_intrinsic1(1,2) = _cy;

	for (int i=0; i<5; i++)
		m_distortion1(i) = m_distortion[i];

   
}

CameraCalibration::CameraCalibration(float _fx, float _fy, float _cx, float _cy, float distortionCoeff[5]) : m_intrinsic(3,3,CV_32FC1), m_extrinsic(4,4,CV_32FC1)
{
	fx() = _fx;
	fy() = _fy;
	cx() = _cx;
	cy() = _cy;

	m_distortion.resize(5);
	for(int i=0; i<5; i++)
		m_distortion[i] = distortionCoeff[i];

	//
	m_intrinsic1 = cv::Matx33f::eye();
	m_distortion1 = cv::Matx<float,5,1>::zeros();
	m_intrinsic1(0,0) = _fx;
	m_intrinsic1(1,1) = _fy;
	m_intrinsic1(0,2) = _cx;
	m_intrinsic1(1,2) = _cy;

	for (int i=0; i<5; i++)
		m_distortion1(i) = m_distortion[i];
}

const Mat& CameraCalibration::getIntrinsic() const
{
	return m_intrinsic;
}

const cv::Matx33f& CameraCalibration::getIntrinsic1() const
{
	return m_intrinsic1;

}

const vector<float>& CameraCalibration::getDistorsion() const
{
	return m_distortion;
}

const cv::Matx<float,5,1>& CameraCalibration::getDistorsion1() const
{
	return m_distortion1;
}

float& CameraCalibration::fx()
{
    return m_intrinsic.at<float>(1,1);
}


float& CameraCalibration::fy()
{
    return m_intrinsic.at<float>(0,0);
}


float& CameraCalibration::cx()
{
    return m_intrinsic.at<float>(0,2);
}


float& CameraCalibration::cy()
{
    return m_intrinsic.at<float>(1,2);
}


float CameraCalibration::fx() const
{
    return m_intrinsic.at<float>(1,1);
}


float CameraCalibration::fy() const
{
    return m_intrinsic.at<float>(0,0);
}


float CameraCalibration::cx() const
{
    return m_intrinsic.at<float>(0,2);
}


float CameraCalibration::cy() const
{
    return m_intrinsic.at<float>(1,2);
}

const Mat& CameraCalibration::getExtrinsic() const
{
	return m_extrinsic;
}

void CameraCalibration::setExtrinsic(Mat& extrinsic)
{
	for(int r=0; r<extrinsic.rows; r++)
	{
		for(int c=0; c<extrinsic.cols; c++)
		{
			m_extrinsic.at<float>(r,c) = extrinsic.at<float>(r,c);
		}
	}
}