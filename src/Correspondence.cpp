#include "stdafx.h"
#include "Correspondence.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include "Param.h"

using namespace ols;

Correspondence::Correspondence(int width, int height)
{
	m_width = width;
	m_height = height;

	int _N = 500;
	int _M = 100;
	this->m_candiPoints.resize(_N);
	for(int i=0; i<_N; i++)
		this->m_candiPoints[i].resize(_M);
}

std::vector<cv::Point> Correspondence::getLine(float k, const cv::Point& center, const cv::Mat& fill_img)
{
	//using static can accelerate 3ms
	static std::vector<cv::Point> points;
	static std::vector<cv::Point> decrease;
	static std::vector<cv::Point> increase;
	points.clear();
	decrease.clear();
	increase.clear();

	/*std::vector<cv::Point> points;
	std::vector<cv::Point> decrease;
	std::vector<cv::Point> increase;*/
	
	float eps = 0;
	int x = center.x;
	int y = center.y;

	if(k <= 1 && k>=0)
	{
		eps = k - 0.5f;
		
		for(int i=0; i<L; i++)
		{
			if(eps >= 0)
			{
				y = y-1; eps = eps-1;//y=y+1
			}
			x = x+1; eps = eps + k;
			if(x<=m_width-1 && y<=m_height-1 && x>=0 && y >=0)
				increase.push_back(cv::Point(x,y));
		}
		eps = k - 0.5f;
		x = center.x;
		y = center.y;
		int t = increase.size()-2*L;
		for(int i=0; i>t; i--)
		{
			if(eps < 0)
			{
				y = y+1; eps = eps+1; //y=y+1
			}
			x = x-1; eps = eps - k;
			if(x<=m_width-1 && y<=m_height-1 && x>=0 && y >=0)
			decrease.push_back(cv::Point(x,y));

		}

		
	}
	if(k>1)
	{		
		eps = 1/k - 0.5f;
		
		for(int i=0; i<L; i++)
		{
			if(eps >= 0)
			{
				x = x-1; eps = eps-1;
			}
			y = y+1; eps = eps + 1/k;
			if(x<=m_width-1 && y<=m_height-1 && x>=0 && y>=0)
				increase.push_back(cv::Point(x,y));
		}
		eps = 1/k - 0.5f;
		x = center.x;
		y = center.y;
		int t = increase.size()-2*L;
		for(int i=0; i>t; i--)
		{
			if(eps < 0)
			{
				x = x+1; eps = eps+1;
			}
			y = y-1; eps = eps-1/k;
			if(x<=m_width-1 && y<=m_height-1 && x>=0 && y >=0)
			decrease.push_back(cv::Point(x,y));
		}
		
	}
	if(k>=-1 && k<0)
	{
		k = fabs(k);
		eps = k - 0.5f;
		
		for(int i=0; i<L; i++)
		{
			if(eps >= 0)
			{
				y = y+1; eps = eps-1;//y=y+1
			}
			x = x+1; eps = eps + k;
			if(x<=m_width-1 && y<=m_height-1 && x>=0 && y>=0)
				increase.push_back(cv::Point(x,y));
		}
		eps = k - 0.5f;
		x = center.x;
		y = center.y;
		int t = increase.size()-2*L;
		for(int i=0; i>t; i--)
		{
			if(eps < 0)
			{
				y = y-1; eps = eps+1; //y=y+1
			}
			x = x-1; eps = eps - k;
			if(x<=m_width-1 && y<=m_height-1 && x>=0 && y >=0)
			decrease.push_back(cv::Point(x,y));

		}
	}
	if(k<-1)
	{
		k = fabs(k);
		eps = 1/k - 0.5f;
		
		for(int i=0; i<L; i++)
		{
			if(eps >= 0)
			{
				x = x+1; eps = eps-1;
			}
			y = y+1; eps = eps + 1/k;
			if(x<=m_width-1 && y<=m_height-1 && x>=0 && y>=0)
				increase.push_back(cv::Point(x,y));
		}
		eps = 1/k - 0.5f;
		x = center.x;
		y = center.y;
		int t = increase.size()-2*L;
		for(int i=0; i>t; i--)
		{
			if(eps < 0)
			{
				x = x-1; eps = eps+1;
			}
			y = y-1; eps = eps-1/k;
			if(x<=m_width-1 && y<=m_height-1 && x>=0 && y >=0)
			decrease.push_back(cv::Point(x,y));
		}
	}

	/*double i_dist = std::pow(increase[increase.size()-1].x-median.x,2.0f)+std::pow(increase[increase.size()-1].y-median.y,2.0f);
	double d_dist = std::pow(decrease[decrease.size()-1].x-median.x,2.0f)+std::pow(decrease[decrease.size()-1].y-median.y,2.0f);*/


	/*uchar i_dist = fill_img.at<uchar>(cv::Point(increase[1].x,increase[1].y));
	uchar d_dist = fill_img.at<uchar>(cv::Point(decrease[1].x,decrease[1].y));*/

	uchar i_dist = 0, d_dist = 0;
	if(increase.size() > 1 && decrease.size() > 1)
	{	
		i_dist = fill_img.at<uchar>(cv::Point(increase[1].x,increase[1].y));
		d_dist = fill_img.at<uchar>(cv::Point(decrease[1].x,decrease[1].y));
	}
	else if(increase.size() > 1 && decrease.size() == 0)
	{
		i_dist = fill_img.at<uchar>(cv::Point(increase[1].x,increase[1].y));
		if(i_dist > 0)
			d_dist = 0;
		else
			d_dist = 255;
	}
	else if(increase.size() ==0 && decrease.size() > 1)
	{
		d_dist = fill_img.at<uchar>(cv::Point(decrease[1].x,decrease[1].y));
		if(d_dist > 0)
			i_dist = 0;
		else
			i_dist = 255;
	}

	if(i_dist>d_dist)  //decrease-center-increase
	{
		for(int i=decrease.size()-1; i>=0; i--)
			points.push_back(decrease[i]);
			
		points.push_back(center);

		for(int i=0; i<increase.size(); i++)
			points.push_back(increase[i]);
	}
	else
	{
		for(int i=increase.size()-1; i>=0; i--)
			points.push_back(increase[i]);
			
		points.push_back(center);

		for(int i=0; i<decrease.size(); i++)
			points.push_back(decrease[i]);
	}

	return points;
}


//for FindCorrespondingPointsInImage
void Correspondence::findSearchLine(PointSet& pointset, cv::Mat& curFrame)
{
	
	int64 time0 = cv::getTickCount();
	//Bresenham's line drawing algorithem (8-connectivity)
	int size = (int)pointset.m_img_points.size();
	//m_search_points.clear();
	m_search_points.resize(size);  //没有clear，会不会有问题，clear语句消耗很多时间
	
	//center point
	int x = 0;
	int y = 0;
	for(int i=0; i<size; i++)
	{
		x += pointset.m_img_points[i].x;
		y += pointset.m_img_points[i].y;
	}
	cv::Point center(x/size,y/size);
	struct Index{
		double dist;
		int idx;
	};
	Index index[1000];
	

	//std::vector<float> slope(size);
	cv::Point p1, p2, p3;

	for(int i=0; i<size; i++)
	{
		float k = 0.0f;

		//method 1: just use near two points
		/*if(i==0)
		k = (pointset.m_img_points[1].y-pointset.m_img_points[size-1].y)/(pointset.m_img_points[1].x-pointset.m_img_points[size-1].x+0.001f);
		else if(i<size-1)
		k = (pointset.m_img_points[i+1].y-pointset.m_img_points[i-1].y)/(pointset.m_img_points[i+1].x-pointset.m_img_points[i-1].x+0.001f);
		else
		k = (pointset.m_img_points[0].y-pointset.m_img_points[i-1].y)/(pointset.m_img_points[0].x-pointset.m_img_points[i-1].x+0.001f);*/
		
		//method 2: interpolate

		if(i == 0)
		{

			p1.x = pointset.m_img_points[0].x - pointset.m_img_points[1].x;
			p1.y = pointset.m_img_points[0].y - pointset.m_img_points[1].y;
			p2.x = pointset.m_img_points[size-1].x - pointset.m_img_points[0].x;
			p2.y = pointset.m_img_points[size-1].y - pointset.m_img_points[0].y;

			float d1 = std::pow(pointset.m_img_points[1].y-pointset.m_img_points[0].y,2.0f)+std::pow(pointset.m_img_points[1].x-pointset.m_img_points[0].x,2.0f);

			float d2 = std::pow(pointset.m_img_points[0].y-pointset.m_img_points[size-1].y,2.0f)+std::pow(pointset.m_img_points[0].x-pointset.m_img_points[size-1].x,2.0f);

			d1 /= 1;
			d2 /= 1;

			p3.x = p1.x*d1 + p2.x*d2;
			p3.y = p1.y*d1 + p2.y*d2;

			k = p3.y/(p3.x+0.0001f);

		}
		else if (i<size-1)
		{
			p1.x = pointset.m_img_points[i+1].x - pointset.m_img_points[i].x;
			p1.y = pointset.m_img_points[i+1].y - pointset.m_img_points[i].y;
			p2.x = pointset.m_img_points[i].x - pointset.m_img_points[i-1].x;
			p2.y = pointset.m_img_points[i].y - pointset.m_img_points[i-1].y;

			float d1 = std::pow(p1.x,2.0f) + std::pow(p1.y,2.0f);
			float d2 = std::pow(p2.x,2.0f) + std::pow(p2.y,2.0f);

			d1 /= 1;
			d2 /= 1;

			p3.x = p1.x*d1 + p2.x*d2;
			p3.y = p1.y*d1 + p2.y*d2;

			k = p3.y/(p3.x+0.0001f);
		}
		else
		{
			p1.x = pointset.m_img_points[0].x - pointset.m_img_points[i].x;
			p1.y = pointset.m_img_points[0].y - pointset.m_img_points[i].y;
			p2.x = pointset.m_img_points[i].x - pointset.m_img_points[i-1].x;
			p2.y = pointset.m_img_points[i].y - pointset.m_img_points[i-1].y;

			float d1 = std::pow(p1.x,2.0f) + std::pow(p1.y,2.0f);
			float d2 = std::pow(p2.x,2.0f) + std::pow(p2.y,2.0f);

			d1 /= 1;
			d2 /= 1;

			p3.x = p1.x*d1 + p2.x*d2;
			p3.y = p1.y*d1 + p2.y*d2;

			k = p3.y/(p3.x+0.0001f);
		}


		//method 3; use the near distance two points 

		//for(int j=0; j<size; j++)
		//{
		//	index[j].dist = std::pow(pointset.m_img_points[i].x-pointset.m_img_points[j].x,2.0f)+std::pow(pointset.m_img_points[i].y-pointset.m_img_points[j].y,2.0f);	
		//	index[j].idx = j;
		//}
		////sort dist from small to big
		//for(int j=0; j<size; j++)
		//{
		//	for(int k=j+1; k<size; k++)
		//	{
		//		if(index[j].dist>index[k].dist)
		//		{
		//			Index tmp;
		//			tmp.dist = index[j].dist;
		//			tmp.idx = index[j].idx;
		//			index[j].dist = index[k].dist;
		//			index[j].idx = index[k].idx;
		//			index[k].dist = tmp.dist;
		//			index[k].idx = tmp.idx;
		//		}
		//	}
		//}
		//int indicate = 1;  //why from 1, because the 0 is itself

		////from the nearest two points
		//if(size > 1)
		//	k = (pointset.m_img_points[index[indicate].idx].y-pointset.m_img_points[index[indicate+1].idx].y)/(pointset.m_img_points[index[indicate].idx].x-pointset.m_img_points[index[indicate+1].idx].x+0.001f);
		//else
		//	k = 1;

		m_search_points[i] = getLine(1/k,pointset.m_img_points[i],pointset.m_fill_object);


	}
	
	

	int64 time1 = cv::getTickCount();
	//printf("line inner time:%lf\n",(time1-time0)/cv::getTickFrequency());

	
	this->N = size;
	this->W = 2*L+1;
	//this->m_lineBundle.create(N,W,CV_8UC3);
	//this->m_candiPoints.resize(N);
	this->m_num.resize(N);
	int64 time_0 = cv::getTickCount();
	//printf("..................................:%d\n",N);
#pragma omp parallel for
	for(int i=0; i<N; i++)
	{
		//this->m_candiPoints[i].resize(W);
		for(int j=0; j<(int)m_search_points[i].size(); j++)
		{
			this->m_lineBundle.at<cv::Vec3b>(i,j) = curFrame.at<cv::Vec3b>(m_search_points[i][j].y,m_search_points[i][j].x);
			this->m_candiPoints[i][j].imgLocation = m_search_points[i][j];
			this->m_candiPoints[i][j].isCandi = false;
			this->m_candiPoints[i][j].score = 0.0;
			this->m_candiPoints[i][j].isMaxValue = false;
			this->m_candiPoints[i][j].energy = -1000.0;
			this->m_candiPoints[i][j].from_location = cv::Point(0,0);
			this->m_candiPoints[i][j].score1 = 0.0;
		}
		/*this->m_lineBundle.at<cv::Vec3b>(i,m_search_points[i].size()/2)[0] = 0;
		this->m_lineBundle.at<cv::Vec3b>(i,m_search_points[i].size()/2)[1] = 0;
		this->m_lineBundle.at<cv::Vec3b>(i,m_search_points[i].size()/2)[2] = 255;*/
	}
	int64 time_1 = cv::getTickCount();
	//printf("malloc time:%lf\n",(time_1-time_0)/cv::getTickFrequency());
	//this->m_hsv.create(N,W,CV_8UC3);
	cv::cvtColor(m_lineBundle,m_hsv,CV_BGR2HSV_FULL);
	//cv::imwrite("line.png",m_lineBundle);
	//cv::imshow("bundle",this->m_lineBundle);
	//cv::waitKey();
}

void Correspondence::updateHist()
{

	for(int i=0; i<Nh*Ns+Nv; i++)
	{
		posHist[i] = 0;
		negHist[i] = 0;
	}
	int pCount = 0, nCount = 0;
	for(int i=0; i<N; i++)
	{
		for(int j=0; j<col[i]; j++)
		{
			int bh = m_hsv.at<cv::Vec3b>(i,j)[0]*Nh/256;
			int bs = m_hsv.at<cv::Vec3b>(i,j)[1]*Ns/256;
			int bv = m_hsv.at<cv::Vec3b>(i,j)[2]*Nv/256;
			//if(bs > 25)
			negHist[bh+BIN*bs]++;
			//if(bv > 50)
			//negHist[BIN*BIN+bv]++;
			nCount++;
		}

		for(int j=col[i]; j<W; j++)
		{
			int bh = m_hsv.at<cv::Vec3b>(i,j)[0]*Nh/256;
			int bs = m_hsv.at<cv::Vec3b>(i,j)[1]*Ns/256;
			int bv = m_hsv.at<cv::Vec3b>(i,j)[2]*Nv/256;
			//if(bs > 25)
			posHist[bh+BIN*bs]++;
			//if(bv > 50)
			//posHist[BIN*BIN+bv]++;
			pCount++;
		}
	}
	for(int i=0; i<Nh*Ns+Nv; i++)
	{
		posHist[i] /= pCount;
		negHist[i] /= nCount;
	}
}

void Correspondence::initHist()
{

	for(int i=0; i<Nh*Ns+Nv; i++)
	{
		posHist[i] = 0;
		negHist[i] = 0;
	}
	int pCount = 0, nCount = 0;
	for(int i=0; i<N; i++)
	{
		for(int j=0; j<W/2; j++)
		{
			int bh = m_hsv.at<cv::Vec3b>(i,j)[0]*Nh/256;
			int bs = m_hsv.at<cv::Vec3b>(i,j)[1]*Ns/256;
			int bv = m_hsv.at<cv::Vec3b>(i,j)[2]*Nv/256;
			//if(bs > 25)
			negHist[bh+BIN*bs]++;
			//if(bv > 50)
			//negHist[BIN*BIN+bv]++;
			nCount++;
		}

		for(int j=W/2; j<W; j++)
		{
			int bh = m_hsv.at<cv::Vec3b>(i,j)[0]*Nh/256;
			int bs = m_hsv.at<cv::Vec3b>(i,j)[1]*Ns/256;
			int bv = m_hsv.at<cv::Vec3b>(i,j)[2]*Nv/256;
			//if(bs > 25)
			posHist[bh+BIN*bs]++;
			//if(bv > 50)
			//posHist[BIN*BIN+bv]++;
			pCount++;
		}
	}
	for(int i=0; i<Nh*Ns+Nv; i++)
	{
		posHist[i] /= pCount;
		negHist[i] /= nCount;
	}
}

void Correspondence::computeCandidates()
{
	cv::Mat gradient(N,W,CV_32SC1,cv::Scalar(0,0,0,0));

	for(int i=0; i<N; i++)
	{
		for(int j=1; j<W-1; j++)
		{
			int g_b = (int)fabs((float)m_hsv.at<cv::Vec3b>(i,j+1)[0] - m_hsv.at<cv::Vec3b>(i,j-1)[0]);
			int g_g = (int)fabs((float)m_hsv.at<cv::Vec3b>(i,j+1)[1] - m_hsv.at<cv::Vec3b>(i,j-1)[1]);
			int g_r = (int)fabs((float)m_hsv.at<cv::Vec3b>(i,j+1)[2] - m_hsv.at<cv::Vec3b>(i,j-1)[2]);

			gradient.at<int>(i,j) = std::max(std::max(g_b,g_g),g_r);

		}
	}

	//compute 1D non-maximum suppression (3-neighbor)
	for(int i=0; i<N; i++)
	{
		for(int j=1; j<W-2; j = j+2)
		{
			if(gradient.at<int>(i,j) > gradient.at<int>(i,j+1))
			{
				if(gradient.at<int>(i,j) > gradient.at<int>(i,j-1))
				{
					if(gradient.at<int>(i,j) > 40)
						m_candiPoints[i][j].isCandi = true;
				}
			}
			else
			{
				j = j+1;
				while(gradient.at<int>(i,j) <= gradient.at<int>(i,j+1) && j < W-2)
				{
					j = j+1;
					
				}
				if(j < W-1)
				{
					if(gradient.at<int>(i,j) > 40)
						m_candiPoints[i][j].isCandi = true;
				}
			}
			
		}
	}
}

void Correspondence::evaluateCandidates()  //the probability belongs to negative
{

	cv::Point preLocation(0,0);
	for(int i=0; i<N; i++)                //row
	{
		
		preLocation.x = 0; preLocation.y = i;
		for(int j=0; j<W; j++)            //col
		{
			if(m_candiPoints[i][j].isCandi == true)
			{
				//compute the histogram between last candidate point to current candidate point
				cv::Point curLocation(j,i);
				double hist[Nh*Ns+Nv] = {0};
				int count = 0;
			
				for(int k=preLocation.x; k<curLocation.x; k++)
				{

					int bh = m_hsv.at<cv::Vec3b>(i,k)[0]*Nh/256;
					int bs = m_hsv.at<cv::Vec3b>(i,k)[1]*Ns/256;
					int bv = m_hsv.at<cv::Vec3b>(i,k)[2]*Nv/256;
					//if(bs > 25)
					hist[bh+BIN*bs]++;
					//if(bv > 50)
					//hist[BIN*BIN+bv]++;
					count++;

				}
				
				for(int k=0; k<Nh*Ns+Nv; k++)
					hist[k] /= count;

				//compare the histogram, get a score
				double pos=0, neg=0;
				for(int k=0; k<Nh*Ns+Nv; k++)
				{
					pos += std::sqrt(posHist[k]*hist[k]);
					neg += std::sqrt(negHist[k]*hist[k]);
				}
				pos = 1 - pos;   //正的取反，负的不取反
				if(pos < 0.3)  //tao
				{
					m_candiPoints[i][j].score = pos;
				}
				else
				{
					m_candiPoints[i][j].score = neg;
				}
				
				//file<<std::setw(10)<<m_candiPoints[i][j].score<<" ";

				preLocation = curLocation;
				//printf("%f\t",m_candiPoints[i][j].score);
				//m_num[i]++;
				
			}
			else
			{
				m_candiPoints[i][j].score = 0;
				
			}
			
		}

	}


}

void Correspondence::evaluateCandidates4DP()
{

	cv::Point nextLocation(0,0);
	for(int i=0; i<N; i++)                //row
	{

		nextLocation.x = W-1; nextLocation.y = i;
		for(int j=W-1; j>=0; j--)            //col
		{
			if(m_candiPoints[i][j].isCandi == true)
			{
				//compute the histogram between last candidate point to current candidate point
				cv::Point curLocation(j,i);
				double hist[Nh*Ns+Nv] = {0};
				int count = 0;

				for(int k=curLocation.x; k<nextLocation.x; k++)
				{

					int bh = m_hsv.at<cv::Vec3b>(i,k)[0]*Nh/256;
					int bs = m_hsv.at<cv::Vec3b>(i,k)[1]*Ns/256;
					int bv = m_hsv.at<cv::Vec3b>(i,k)[2]*Nv/256;
					//if(bs > 25)
					hist[bh+BIN*bs]++;
					//if(bv > 50)
					//hist[BIN*BIN+bv]++;
					count++;

				}

				for(int k=0; k<Nh*Ns+Nv; k++)
					hist[k] /= count;

				//compare the histogram, get a score
				double pos=0, neg=0;
				for(int k=0; k<Nh*Ns+Nv; k++)
				{
					pos += std::sqrt(posHist[k]*hist[k]);
					neg += std::sqrt(negHist[k]*hist[k]);
				}
				neg = 1 - neg;   //正的取反，负的不取反
				if(neg > 0.7)  //tao
				{
					m_candiPoints[i][j].score1 = neg;
				}
				else
				{
					m_candiPoints[i][j].score1 = pos;
				}

				//file<<std::setw(10)<<m_candiPoints[i][j].score1<<" ";
				nextLocation = curLocation;
				//printf("%f\t",m_candiPoints[i][j].score);
				//m_num[i]++;

			}
			else
			{
				m_candiPoints[i][j].score1 = 0;
				//file<<std::setw(10)<<0;
			}

		}
		//printf("\n");
		//file<<"\n";
	}

}
/*************************************by dynamic programmingn (our new method)*********************/
void Correspondence::FindCorrespondingPointsByDP(cv::Mat& frame, PointSet& pointset)
{
	if(pointset.m_img_points.size() > 0)
	{
		int64 time0 = cv::getTickCount();
		//m_candiPoints.clear();
		findSearchLine(pointset,frame);
		int64 time1 = cv::getTickCount();
		//printf("find search line time:%lf\n",(time1-time0)/cv::getTickFrequency());

		
		static bool flag = true;
		if(flag == true)
			this->initHist();
		flag = false;
		this->computeCandidates();
		int64 time_0 = cv::getTickCount();
		this->evaluateCandidates();
		this->evaluateCandidates4DP();
		
		//this->findLongestPath(pointset,frame);
		this->findLongestPathDP(pointset,frame);
		
		int64 time_1 = cv::getTickCount();
		//printf("candidate time:%lf\n",(time_1-time_0)/cv::getTickFrequency());
		//draw the lines
		cv::Mat tmp = frame.clone();
		for(int i=0; i<m_search_points.size(); i++)
		{
			for(int j=0; j<m_search_points[i].size(); j++)
			{
				/*tmp.at<cv::Vec3b>(m_search_points[i][j])[0] = 255;
				tmp.at<cv::Vec3b>(m_search_points[i][j])[1] = 255;
				tmp.at<cv::Vec3b>(m_search_points[i][j])[2] = 255;*/
				cv::line(tmp,m_search_points[i][0],m_search_points[i][m_search_points[i].size()-1],cv::Scalar(0,255,0),1);

			}

		}

		//////display the candidate of the correspondence, the white dot is the correspondence
		for(int i=0; i<N; i++)
		{
			for(int j=0; j<W; j++)
			{
				if(this->m_candiPoints[i][j].isCandi == true)
				{
					if(m_candiPoints[i][j].isMaxValue == true)
					{
						m_lineBundle.at<cv::Vec3b>(i,j)[0] = 255;
						m_lineBundle.at<cv::Vec3b>(i,j)[1] = 255;
						m_lineBundle.at<cv::Vec3b>(i,j)[2] = 255;
						
						//cv::circle(tmp,m_candiPoints[i][j].imgLocation,2,cv::Scalar(0,0,255),2);

					}
					else
					{
						m_lineBundle.at<cv::Vec3b>(i,j)[0] = 0;
						m_lineBundle.at<cv::Vec3b>(i,j)[1] = 0;
						m_lineBundle.at<cv::Vec3b>(i,j)[2] = 255;
					}
				}
			}
		}
		////cv::imshow("correspondence",m_lineBundle);
		//////cv::imshow("hsv space",m_hsv);
		cv::imshow("frame",tmp);
		//cv::waitKey();
		//cv::imwrite("small.png",m_lineBundle);
	}
}

inline double Correspondence::computeProb(const CandidatePoint& point)
{
	double weight;
	weight = std::exp((point.score+point.score1)/10.0);
	return weight;
}

inline double Correspondence::computeEdgeWeight(const CandidatePoint& curCandiPoint, const CandidatePoint& preCandiPoint, const cv::Mat& frame)
{
	double spatial_dist = std::pow(curCandiPoint.imgLocation.x - preCandiPoint.imgLocation.x,2.0)+std::pow(curCandiPoint.imgLocation.y - preCandiPoint.imgLocation.y,2.0);
	double weight = std::exp(-spatial_dist/1000.0);//*std::exp(-std::pow((frame.at<cv::Vec3b>(curCandiPoint.imgLocation)[0]-frame.at<cv::Vec3b>(preCandiPoint.imgLocation)[0]),2.0)/100.0);;
	return weight;
}

void Correspondence::findLongestPathDP(PointSet& pointset, const cv::Mat& frame)
{
	cv::Point index_max_location;  //index the max location, row indicates x, col indicates y

	//the first layer from the source node
	int s_num = 0;
	for(int j=0; j<W; j++)
	{
		if(m_candiPoints[0][j].isCandi == true)
		{
			s_num++;
			m_candiPoints[0][j].energy = computeProb(m_candiPoints[0][j]);
			m_candiPoints[0][j].from_location = cv::Point(0,j);
					
		}
	}
	//the first layer check whether it has candidate or not, if not
	if(s_num == 0)
	{
		m_candiPoints[0][W/2].energy = computeProb(m_candiPoints[0][W/2]);
		m_candiPoints[0][W/2].from_location = cv::Point(0,W/2);
	}

	//the other layers
	for(int i=1; i<N; i++)
	{		
		//judge if it has candidate or not, if not, using point(i,W/2) as candidate
		int num_candi = 0;
		for(int j=0; j<W; j++)
		{
			if(m_candiPoints[i][j].isCandi == true)
				num_candi++;
		}
		if(num_candi == 0)
		{
			m_candiPoints[i][W/2].isCandi = true;	
		}
		//end judge

		for(int j=0; j<W; j++)
		{					
			double max_energy = -1000;
			cv::Point location;

			if(m_candiPoints[i][j].isCandi == true)
			{			
				
				m_candiPoints[i][j].energy = computeProb(m_candiPoints[i][j]);	
			
				for(int k=0; k<W; k++)
				{
					
					if(m_candiPoints[i-1][k].isCandi == true)
					{
						
						double edge_weight = computeEdgeWeight(m_candiPoints[i][j],m_candiPoints[i-1][k],frame);
						double energy = m_candiPoints[i][j].energy + m_candiPoints[i-1][k].energy + edge_weight;
						if(max_energy < energy)
						{
							max_energy = energy;
							
							location = cv::Point(i-1,k);
							
						}
						
					}

				}	
			
				m_candiPoints[i][j].energy  = max_energy;  //update the energy
				m_candiPoints[i][j].from_location = location;
				index_max_location = cv::Point(i,j);
			}	

		}

	}
	//the terminate node from the last node
	double max_energy = -1000.0;
	for(int j=0; j<W; j++)
	{
		if(m_candiPoints[N-1][j].isCandi == true)
		{
			if(max_energy < m_candiPoints[N-1][j].energy)
			{
				max_energy = m_candiPoints[N-1][j].energy;
				index_max_location = cv::Point(N-1,j);
			}

		}
	}

	//back tracking find best path
	pointset.m_correspoinding_points.resize(N);
	cv::Point from_location(0,0);
	
	pointset.m_correspoinding_points[N-1] = m_candiPoints[N-1][index_max_location.y].imgLocation;
	from_location = m_candiPoints[N-1][index_max_location.y].from_location;
	m_candiPoints[N-1][index_max_location.y].isMaxValue = true;
	col[N-1] = index_max_location.y;

	
	for(int i=N-2; i>=0; i--)
	{
		if(from_location.x == i)  //x indicates row, somehow it is different from OpenCV
		{
			pointset.m_correspoinding_points[i] = m_candiPoints[i][from_location.y].imgLocation;
			col[i] = from_location.y;
			from_location = m_candiPoints[i][from_location.y].from_location;
			m_candiPoints[i][col[i]].isMaxValue = true;
		}

	}
}


/***************************************by OLS' paper*****************************/
void Correspondence::FindCorrespondingPointsInImage(cv::Mat& curframe, PointSet& pointset)
{
	if(pointset.m_img_points.size() > 0)
	{
		//m_candiPoints.clear();
		findSearchLine(pointset,curframe);

		
		static bool flag = true;
		if(flag == true)
			this->initHist();
		flag = false;
		this->computeCandidates();
		this->evaluateCandidates();
		this->searchCorrespondencePoints(pointset);

		//draw the lines
		/*cv::Mat frame = curframe.clone();
		for(int i=0; i<m_search_points.size(); i++)
		{
		for(int j=0; j<m_search_points[i].size(); j++)
		{
		frame.at<cv::Vec3b>(m_search_points[i][j])[0] = 255;
		frame.at<cv::Vec3b>(m_search_points[i][j])[1] = 255;
		frame.at<cv::Vec3b>(m_search_points[i][j])[2] = 255;

		}
		frame.at<cv::Vec3b>(pointset.m_img_points[i])[0] = 0;
		frame.at<cv::Vec3b>(pointset.m_img_points[i])[1] = 0;
		frame.at<cv::Vec3b>(pointset.m_img_points[i])[2] = 255;
		}
		cv::imshow("frame",frame);*/
	}
}

void Correspondence::searchCorrespondencePoints(PointSet& pointset)
{

	for(int i=0; i<N; i++)
	{
		m_num[i] = 0;
		for(int j=0; j<W; j++)
		{
			if(m_candiPoints[i][j].isCandi == true)
			{
				if(m_candiPoints[i][j].score < 0.35)    //delete the object clutter according to the histogram matching, score is the value of belonging to negative, different video needs different value; 0.15效果太好了!!!
					m_candiPoints[i][j].isCandi = false;
				else
					m_num[i]++;
			}
		}
	}

	for(int i=0; i<N; i++)
	{
		if(m_num[i] > 0)  //the num of candidate >= 1
		{
			for(int j=W-1; j>=0; j--)
			{
				if(m_candiPoints[i][j].isCandi == true)
				{
					/*if(fabs((float)pointset.m_img_points[i].x - m_candiPoints[i][j].imgLocation.x)>5.0 && fabs((float)pointset.m_img_points[i].y-m_candiPoints[i][j].imgLocation.y)>5.0)
						pointset.m_correspoinding_points.push_back(pointset.m_img_points[i]);
					else*/
						pointset.m_correspoinding_points.push_back(m_candiPoints[i][j].imgLocation);
					m_candiPoints[i][j].isMaxValue = true;
					col[i] = j;
					break;
				}
			}
		}
		else
		{
			pointset.m_correspoinding_points.push_back(m_candiPoints[i][W/2].imgLocation);
			//pointset.m_correspoinding_points.push_back(pointset.m_img_points[i]);
			m_candiPoints[i][W/2].isMaxValue = true;
			m_candiPoints[i][W/2].isCandi = true;
			col[i] = W/2;
		}
	}

	//display the candidate of the correspondence, the white dot is the correspondence
	for(int i=0; i<N; i++)
	{
		for(int j=0; j<W; j++)
		{
			if(this->m_candiPoints[i][j].isCandi == true)
			{
				if(m_candiPoints[i][j].isMaxValue == true)
				{
					m_lineBundle.at<cv::Vec3b>(i,j)[0] = 255;
					m_lineBundle.at<cv::Vec3b>(i,j)[1] = 255;
					m_lineBundle.at<cv::Vec3b>(i,j)[2] = 255;


				}
				else
				{
					m_lineBundle.at<cv::Vec3b>(i,j)[0] = 0;
					m_lineBundle.at<cv::Vec3b>(i,j)[1] = 0;
					m_lineBundle.at<cv::Vec3b>(i,j)[2] = 255;
				}
			}
		}
	}
	cv::imshow("correspondence",m_lineBundle);
	//cv::imwrite("local.png",m_lineBundle);
	//cv::waitKey();
}