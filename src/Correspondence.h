#ifndef _CORRESPONDENCE_H
#define _CORRESPONDENCE_H
#include "PointSet.h"
#include <opencv2\core\core.hpp>
namespace ols
{

#define BIN 4
#define Nh BIN
#define Ns BIN
#define Nv 0


	struct CandidatePoint
	{
		cv::Point imgLocation;
		double score;  //compare with the previous location
		bool isCandi;
		bool isMaxValue;

		//for dynamic programming
		double energy;
		cv::Point from_location; //previous note to current note: energy minimization
		double score1; //compare with the next location
		
	};

	class Correspondence
	{
	public:
		Correspondence(int width, int height);
	public:
		void FindCorrespondingPointsByDP(cv::Mat& frame, PointSet& pointset);
		void FindCorrespondingPointsInImage(cv::Mat& curframe, PointSet& pointset);
	private:
		void findSearchLine(PointSet& pointset, cv::Mat& curFrame);
		std::vector<cv::Point> getLine(float k, const cv::Point& center, const cv::Mat& fill_img);
		void computeCandidates();
		void evaluateCandidates();
		void initHist();

	private:
		void searchCorrespondencePoints(PointSet& pointset);
	public:
		void updateHist();
		cv::Mat m_lineBundle;
		cv::Mat m_hsv;
	private:
		int m_width;
		int m_height;
		std::vector<std::vector<cv::Point>> m_search_points;
		double posHist[Nh*Ns+Nv];
		double negHist[Nh*Ns+Nv];
		
		std::vector<int> m_num;
		int N;   //height
		int W;   //width
		int col[1000];  //used for updating the histogram
		std::vector<std::vector<CandidatePoint>> m_candiPoints;

		//weight for dynamic programming
		void findLongestPathDP(PointSet& pointset, const cv::Mat& frame);
		void evaluateCandidates4DP();
		inline double computeProb(const CandidatePoint& point);
		inline double computeEdgeWeight(const CandidatePoint& curCandiPoint, const CandidatePoint& preCandiPoint, const cv::Mat& frame);
		
	};
}
#endif