#ifndef _OPTIMIZER_H
#define _OPTIMIZER_H
#include <opencv2\core\core.hpp>
#include <opencv2\video\tracking.hpp>
#include "PointSet.h"
#include "Config.h"
#include "Model.h"
#include "Correspondence.h"
#include <fstream>

namespace ols
{
	struct Data
	{
		PointSet m_pointset;
		Model* m_model;
		Correspondence* m_correspondence;
		int m_n;

		cv::Mat m_frame;

		//for robust m-estimation
		float m_weight[1000];
		float m_residual[1000];
		float m_jac[1000][6];
	};

	

	class Optimizer
	{
	public:
		Optimizer(const Config& config, const cv::Mat& initPose, bool is_writer);
		~Optimizer();
	public:
		cv::Mat optimizingLM(cv::Mat& prepose, cv::Mat& frame, int frameId);
		void computeExtrinsic(const cv::Mat& prepose, cv::Mat& extrinsic);
	private:
		static void lm(double *p, double* x, int m, int n, void* data);	
		static void jaclm(double *p, double *jac, int m, int n, void* data);

	public:
		Data m_data;	
	private:

		bool m_is_writer;
		std::ofstream m_outPose;
		CameraCalibration m_calibration;

	};
}
#endif