#ifndef _CAMERA_CALIBRATION_H
#define _CAMERA_CALIBRATION_H

#include <opencv2\core\core.hpp>
#include <vector>
using namespace std;
using namespace cv;

namespace ols
{
	class CameraCalibration
	{
	public:
		CameraCalibration();
		CameraCalibration(float fx, float fy, float cx, float cy);
		CameraCalibration(float fx, float fy, float cx, float cy, float distortionCoeff[5]);

		float& fx();
		float& fy();


		float& cx();
		float& cy();


		float fx() const;
		float fy() const;


		float cx() const;
		float cy() const;

		const Mat& getIntrinsic() const;
		const vector<float>& getDistorsion() const;
		const Mat& getExtrinsic() const;
		void setExtrinsic(Mat& extrinsic);

		const cv::Matx33f& getIntrinsic1() const;
		const cv::Matx<float,5,1>&  getDistorsion1() const;

	private:
		Mat m_intrinsic;
		vector<float> m_distortion;
		Mat m_extrinsic;

		cv::Matx33f     m_intrinsic1;
		cv::Matx<float,5,1>     m_distortion1;
	};
}

#endif