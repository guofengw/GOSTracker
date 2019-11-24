#include "stdafx.h"
#include <iostream>
#include "Model.h"
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv\cv.h>
#include <vector>
#include "Render.h"
#include "Param.h"
#include <omp.h>

using namespace ols;

extern osa::Render g_render;

Model::Model(const Config& config) : m_bb_points(4,8,CV_32FC1)
{
	m_model = config.model;
	m_radius = glmMaxRadius(m_model);

	m_calibration = config.camCalibration;
	m_width = config.width;
	m_height = config.height;
	m_rvec.create(3,1,CV_32FC1);
	m_tvec.create(3,1,CV_32FC1);
}

Model::~Model()
{
	if(m_model)
		glmDelete(m_model);
}

void Model::LoadGLMModel(const std::string& filename)
{
	m_model = glmReadOBJ(const_cast<char*>(filename.c_str()));
	m_radius = glmMaxRadius(m_model);
	
	if(!m_model)
		return;
}

GLMmodel* Model::GetObjModel()
{
	return m_model;
}

void Model::GetImagePoints(const cv::Mat& pose, PointSet& pointset)
{
	
	pointset.m_img_points.clear();
	pointset.m_img_points_f.clear();
	float x = pose.at<float>(0,0); float y = pose.at<float>(0,1); float z = pose.at<float>(0,2);
	float rx = pose.at<float>(0,3); float ry = pose.at<float>(0,4); float rz = pose.at<float>(0,5);

#ifdef MY
	//camera intrinsic
	static cv::Mat intrinsic(3,3,CV_32FC1);
	intrinsic.at<float>(0,0)=m_calibration.fx(); intrinsic.at<float>(0,1)=0; intrinsic.at<float>(0,2)=m_calibration.cx(); //intrinsic.at<float>(0,3)=0;
	intrinsic.at<float>(1,0)=0; intrinsic.at<float>(1,1)=m_calibration.fy(); intrinsic.at<float>(1,2)=m_calibration.cy(); //intrinsic.at<float>(1,3)=0;
	intrinsic.at<float>(2,0)=0; intrinsic.at<float>(2,1)=0; intrinsic.at<float>(2,2)=1; //intrinsic.at<float>(2,3)=0;

	//camera extrinsic
	static cv::Mat extrinsic(3,4,CV_32FC1);

	extrinsic.at<float>(0,0)=cos(ry)*cos(rz); extrinsic.at<float>(0,1)=sin(rx)*sin(ry)-cos(rx)*cos(ry)*sin(rz); extrinsic.at<float>(0,2)=cos(rx)*sin(ry)+cos(ry)*sin(rx)*sin(rz); extrinsic.at<float>(0,3)=x;
	extrinsic.at<float>(1,0)=sin(rz); extrinsic.at<float>(1,1)=cos(rx)*cos(rz); extrinsic.at<float>(1,2)=-cos(rz)*sin(rx); extrinsic.at<float>(1,3)=y;
	extrinsic.at<float>(2,0)=-cos(rz)*sin(ry); extrinsic.at<float>(2,1)=cos(ry)*sin(rx)+cos(rx)*sin(ry)*sin(rz); extrinsic.at<float>(2,2)=cos(rx)*cos(ry)-sin(rx)*sin(ry)*sin(rz); extrinsic.at<float>(2,3)=z;
#else
	//camera intrinsic
	static cv::Mat intrinsic(3,4,CV_32FC1);
	intrinsic.at<float>(0,0)=m_calibration.fx(); intrinsic.at<float>(0,1)=0; intrinsic.at<float>(0,2)=m_calibration.cx(); intrinsic.at<float>(0,3)=0;
	intrinsic.at<float>(1,0)=0; intrinsic.at<float>(1,1)=m_calibration.fy(); intrinsic.at<float>(1,2)=m_calibration.cy(); intrinsic.at<float>(1,3)=0;
	intrinsic.at<float>(2,0)=0; intrinsic.at<float>(2,1)=0; intrinsic.at<float>(2,2)=1; intrinsic.at<float>(2,3)=0;

	//camera extrinsic
	/*static cv::Mat extrinsic(3,4,CV_32FC1);

	extrinsic.at<float>(0,0)=cos(ry)*cos(rz); extrinsic.at<float>(0,1)=sin(rx)*sin(ry)-cos(rx)*cos(ry)*sin(rz); extrinsic.at<float>(0,2)=cos(rx)*sin(ry)+cos(ry)*sin(rx)*sin(rz); extrinsic.at<float>(0,3)=x;
	extrinsic.at<float>(1,0)=sin(rz); extrinsic.at<float>(1,1)=cos(rx)*cos(rz); extrinsic.at<float>(1,2)=-cos(rz)*sin(rx); extrinsic.at<float>(1,3)=y;
	extrinsic.at<float>(2,0)=-cos(rz)*sin(ry); extrinsic.at<float>(2,1)=cos(ry)*sin(rx)+cos(rx)*sin(ry)*sin(rz); extrinsic.at<float>(2,2)=cos(rx)*cos(ry)-sin(rx)*sin(ry)*sin(rz); extrinsic.at<float>(2,3)=z;*/
	cv::Mat extrinsic = GetPoseMatrix();
#endif
	
	std::vector<cv::Point3f>& samplePoints = pointset.m_model_points;

	
	
	cv::Mat pos(4,samplePoints.size(),CV_32FC1);
	cv::Mat result(3,samplePoints.size(),CV_32FC1);

	for(int i=0; i<(int)samplePoints.size(); i++)
	{
		pos.at<float>(0,i) = samplePoints[i].x;
		pos.at<float>(1,i) = samplePoints[i].y;
		pos.at<float>(2,i) = samplePoints[i].z;
		pos.at<float>(3,i) = 1;
	}
	
	result = intrinsic*extrinsic*pos;
	for(int i=0; i<samplePoints.size(); i++)
	{
		float u = result.at<float>(0,i)/result.at<float>(2,i);
		float v = result.at<float>(1,i)/result.at<float>(2,i);
		if(u>=0 && u<m_width && v>=0 && v<m_height)
		{
			pointset.m_img_points.push_back(cv::Point(u,v));
			pointset.m_img_points_f.push_back(cv::Point2f(u,v));
		}
	}


}

PointSet& Model::GetVisibleModelPointsCV(const cv::Mat& prepose, int pointnum)
{
	m_point_set.Clear();
	FilterModel(prepose,pointnum);
	return m_point_set;
}


static inline GLvoid
glmCross(GLfloat* u, GLfloat* v, GLfloat* n)
{
    assert(u); assert(v); assert(n);
    
    n[0] = u[1]*v[2] - u[2]*v[1];
    n[1] = u[2]*v[0] - u[0]*v[2];
    n[2] = u[0]*v[1] - u[1]*v[0];
}

/* glmNormalize: normalize a vector
 *
 * v - array of 3 GLfloats (GLfloat v[3]) to be normalized
 */
static inline GLvoid
glmNormalize(GLfloat* v)
{
    GLfloat l;
    
    assert(v);
    
    l = (GLfloat)sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    v[0] /= l;
    v[1] /= l;
    v[2] /= l;
}

void Model::DisplayCV(const cv::Mat& pose, cv::Mat& frame)
{
	float x = pose.at<float>(0,0); float y = pose.at<float>(0,1); float z = pose.at<float>(0,2);
	float rx = pose.at<float>(0,3); float ry = pose.at<float>(0,4); float rz = pose.at<float>(0,5);

#ifdef MY
	//camera intrinsic
	cv::Mat intrinsic(3,3,CV_32FC1);
	intrinsic.at<float>(0,0)=m_calibration.fx(); intrinsic.at<float>(0,1)=0; intrinsic.at<float>(0,2)=m_calibration.cx(); //intrinsic.at<float>(0,3)=0;
	intrinsic.at<float>(1,0)=0; intrinsic.at<float>(1,1)=m_calibration.fy(); intrinsic.at<float>(1,2)=m_calibration.cy(); //intrinsic.at<float>(1,3)=0;
	intrinsic.at<float>(2,0)=0; intrinsic.at<float>(2,1)=0; intrinsic.at<float>(2,2)=1; //intrinsic.at<float>(2,3)=0;

	//camera extrinsic
	cv::Mat extrinsic(3,4,CV_32FC1);
	computeExtrinsicByEuler(&extrinsic,x,y,z,rx,ry,rz);
#else
	//camera intrinsic
	cv::Mat intrinsic(3,4,CV_32FC1);
	intrinsic.at<float>(0,0)=m_calibration.fx(); intrinsic.at<float>(0,1)=0; intrinsic.at<float>(0,2)=m_calibration.cx(); intrinsic.at<float>(0,3)=0;
	intrinsic.at<float>(1,0)=0; intrinsic.at<float>(1,1)=m_calibration.fy(); intrinsic.at<float>(1,2)=m_calibration.cy(); intrinsic.at<float>(1,3)=0;
	intrinsic.at<float>(2,0)=0; intrinsic.at<float>(2,1)=0; intrinsic.at<float>(2,2)=1; intrinsic.at<float>(2,3)=0;

	//camera extrinsic
	cv::Mat extrinsic = GetPoseMatrix();
#endif
	//compute the points in the 3d space
	cv::Mat _pos(4,m_model->numvertices+1,CV_32FC1);
	cv::Mat _result(3,m_model->numvertices+1,CV_32FC1);
	for(int i=0; i<=m_model->numvertices; i++)
	{
		_pos.at<float>(0,i) = m_model->vertices[3*(i)+0];
		_pos.at<float>(1,i) = m_model->vertices[3*(i)+1];
		_pos.at<float>(2,i) = m_model->vertices[3*(i)+2];
		_pos.at<float>(3,i) = 1;
	}
	_result = extrinsic*_pos;

	//compute the norm of facet
	float u[3], v[3], n[3], c[3];
	for (int i = 0; i < m_model->numtriangles; i++) {

		//compute the norm of the triangles
		u[0] = _result.at<float>(0,m_model->triangles[i].vindices[1]) - _result.at<float>(0,m_model->triangles[i].vindices[0]);

		u[1] = _result.at<float>(1,m_model->triangles[i].vindices[1]) - _result.at<float>(1,m_model->triangles[i].vindices[0]);

		u[2] = _result.at<float>(2,m_model->triangles[i].vindices[1]) - _result.at<float>(2,m_model->triangles[i].vindices[0]);

		v[0] = _result.at<float>(0,m_model->triangles[i].vindices[2]) - _result.at<float>(0,m_model->triangles[i].vindices[0]);

		v[1] = _result.at<float>(1,m_model->triangles[i].vindices[2]) - _result.at<float>(1,m_model->triangles[i].vindices[0]);

		v[2] = _result.at<float>(2,m_model->triangles[i].vindices[2]) - _result.at<float>(2,m_model->triangles[i].vindices[0]);
		
		glmCross(u, v, n);
		glmNormalize(n);
		
		//center of triangle
		c[0] = 0.25*_result.at<float>(0,m_model->triangles[i].vindices[0]) + 0.25*_result.at<float>(0,m_model->triangles[i].vindices[1]) + 0.5*_result.at<float>(0,m_model->triangles[i].vindices[2]);
		c[1] = 0.25*_result.at<float>(1,m_model->triangles[i].vindices[0]) + 0.25*_result.at<float>(1,m_model->triangles[i].vindices[1]) + 0.5*_result.at<float>(1,m_model->triangles[i].vindices[2]);
		c[2] = 0.25*_result.at<float>(2,m_model->triangles[i].vindices[0]) + 0.25*_result.at<float>(2,m_model->triangles[i].vindices[1]) + 0.5*_result.at<float>(2,m_model->triangles[i].vindices[2]);

		glmNormalize(c);

		//judge the whether the line is visible or not
		float cross = n[0]*c[0] + n[1]*c[1] + n[2]*c[2];
		if(cross < 0.0f)
		{
			if(m_model->lines[m_model->triangles[i].lindices[0]].e1 == 1)
				m_model->lines[m_model->triangles[i].lindices[0]].e2 = 1;
			else
				m_model->lines[m_model->triangles[i].lindices[0]].e1 = 1;

			if(m_model->lines[m_model->triangles[i].lindices[1]].e1 == 1)
				m_model->lines[m_model->triangles[i].lindices[1]].e2 = 1;
			else
				m_model->lines[m_model->triangles[i].lindices[1]].e1 = 1;

			if(m_model->lines[m_model->triangles[i].lindices[2]].e1 == 1)
				m_model->lines[m_model->triangles[i].lindices[2]].e2 = 1;
			else
				m_model->lines[m_model->triangles[i].lindices[2]].e1 = 1;
		}

	}

	//extract the points which can be visible or in the edge of object
	std::vector<cv::Point> vertexIndexs;
	std::vector<cv::Point3f> points_3d;

	for(int i=0; i<m_model->numLines; i++)
	{
		if((m_model->lines[i].e1 == 1 && m_model->lines[i].e2 == 0) || (m_model->lines[i].e1==0 && m_model->lines[i].e2==1) || (m_model->lines[i].e1==1 && m_model->lines[i].e2==1))
		{
			GLuint v0 = m_model->lines[i].vindices[0];
			GLuint v1 = m_model->lines[i].vindices[1];

			float x0 = m_model->vertices[3*v0];
			float y0 = m_model->vertices[3*v0+1];
			float z0 = m_model->vertices[3*v0+2];

			float x1 = m_model->vertices[3*v1];
			float y1 = m_model->vertices[3*v1+1];
			float z1 = m_model->vertices[3*v1+2];

			points_3d.push_back(cv::Point3f(x0,y0,z0));
			points_3d.push_back(cv::Point3f(x1,y1,z1));
			
			vertexIndexs.push_back(cv::Point(v0,v1));

		}
		m_model->lines[i].e1 = 0; m_model->lines[i].e2 = 0;
	}

	//compute the image coordinate of visible points
	cv::Mat pos(4,2*vertexIndexs.size(),CV_32FC1);
	cv::Mat result(3,2*vertexIndexs.size(),CV_32FC1);

	for(int i=0; i<(int)vertexIndexs.size(); i++)
	{
		pos.at<float>(0,2*i) = points_3d[2*i].x;
		pos.at<float>(1,2*i) = points_3d[2*i].y;
		pos.at<float>(2,2*i) = points_3d[2*i].z;
		pos.at<float>(3,2*i) = 1;
		pos.at<float>(0,2*i+1) = points_3d[2*i+1].x;
		pos.at<float>(1,2*i+1) = points_3d[2*i+1].y;
		pos.at<float>(2,2*i+1) = points_3d[2*i+1].z;
		pos.at<float>(3,2*i+1) = 1;
	}
	result = intrinsic*extrinsic*pos;

	//display the visible lines
	for(int i=0; i<vertexIndexs.size(); i++)
	{
		int u1 = result.at<float>(0,2*i)/result.at<float>(2,2*i);
		int v1 = result.at<float>(1,2*i)/result.at<float>(2,2*i);
		
		int u2 = result.at<float>(0,2*i+1)/result.at<float>(2,2*i+1);
		int v2 = result.at<float>(1,2*i+1)/result.at<float>(2,2*i+1);
		if(u1 >=0 && u1<frame.cols && v1 >=0 && v1<=frame.rows && u2 >=0 && u2<frame.cols && v2>=0 && v2<=frame.rows)
		{
			cv::line(frame,cv::Point(u1,v1),cv::Point(u2,v2),cv::Scalar(0,0,255),1,CV_AA);
		}
	}
}

void Model::DisplayGL(const cv::Mat& prepose)
{
	//compute extrinsic
	g_render.m_shapePoseInfo.clear();
	float x = prepose.at<float>(0,0); float y = prepose.at<float>(0,1); float z = prepose.at<float>(0,2);
	float rx = prepose.at<float>(0,3); float ry = prepose.at<float>(0,4); float rz = prepose.at<float>(0,5);

#ifdef MY
	//camera extrinsic
	cv::Mat extrinsic(4,4,CV_32FC1);
	const float pi = 3.1415926f;
	extrinsic.at<float>(0,0)=cos(ry)*cos(rz); extrinsic.at<float>(0,1)=sin(rx)*sin(ry)-cos(rx)*cos(ry)*sin(rz); extrinsic.at<float>(0,2)=cos(rx)*sin(ry)+cos(ry)*sin(rx)*sin(rz); extrinsic.at<float>(0,3)=x;
	extrinsic.at<float>(1,0)=sin(rz); extrinsic.at<float>(1,1)=cos(rx)*cos(rz); extrinsic.at<float>(1,2)=-cos(rz)*sin(rx); extrinsic.at<float>(1,3)=y;
	extrinsic.at<float>(2,0)=-cos(rz)*sin(ry); extrinsic.at<float>(2,1)=cos(ry)*sin(rx)+cos(rx)*sin(ry)*sin(rz); extrinsic.at<float>(2,2)=cos(rx)*cos(ry)-sin(rx)*sin(ry)*sin(rz); extrinsic.at<float>(2,3)=z;
	extrinsic.at<float>(3,0)=0; extrinsic.at<float>(3,1)=0; extrinsic.at<float>(3,2)=0; extrinsic.at<float>(3,3)=1;

#else
	cv::Mat extrinsic = GetPoseMatrix();
#endif

	osa::ShapePoseInfo shapePoseInfo;
	shapePoseInfo.m_shape = m_model;
	g_render.matrixFromCV2GL(extrinsic,shapePoseInfo.mv_matrix);
	g_render.m_shapePoseInfo.push_back(shapePoseInfo);
	g_render.rendering();

	//cv::Mat tmp = g_render.getRenderedImg();
	////change to 3 channel
	//cv::Mat render_img(tmp.rows,tmp.cols,CV_8UC3);
	//for(int r=0; r<tmp.rows; r++)
	//	for(int c=0; c<tmp.cols; c++)
	//	{
	//		render_img.at<cv::Vec3b>(r,c)[0] = tmp.at<uchar>(r,c);
	//		render_img.at<cv::Vec3b>(r,c)[1] = tmp.at<uchar>(r,c);
	//		render_img.at<cv::Vec3b>(r,c)[2] = tmp.at<uchar>(r,c);
	//	}
	//static cv::VideoWriter writer("result_s7_render.avi",CV_FOURCC('D','I','V','X'),30,render_img.size());
	//writer<<render_img;
}

static void Extrinsic(cv::Mat* rotMatrix, float rx, float ry, float rz)
{
	//ry*rz*rx
	const float pi = 3.1415926f;

	(*rotMatrix).at<float>(0,0)=cos(ry)*cos(rz); (*rotMatrix).at<float>(0,1)=sin(rx)*sin(ry)-cos(rx)*cos(ry)*sin(rz); (*rotMatrix).at<float>(0,2)=cos(rx)*sin(ry)+cos(ry)*sin(rx)*sin(rz);
	(*rotMatrix).at<float>(1,0)=sin(rz); (*rotMatrix).at<float>(1,1)=cos(rx)*cos(rz); (*rotMatrix).at<float>(1,2)=-cos(rz)*sin(rx);
	(*rotMatrix).at<float>(2,0)=-cos(rz)*sin(ry); (*rotMatrix).at<float>(2,1)=cos(ry)*sin(rx)+cos(rx)*sin(ry)*sin(rz); (*rotMatrix).at<float>(2,2)=cos(rx)*cos(ry)-sin(rx)*sin(ry)*sin(rz);

	//ry*rx*rz
	/*(*rotMatrix).at<float>(0,0)=cos(ry)*cos(rz)+sin(rx)*sin(ry)*sin(rz); (*rotMatrix).at<float>(0,1)=cos(rz)*sin(rx)*sin(ry)-cos(ry)*sin(rz); (*rotMatrix).at<float>(0,2)=cos(rx)*sin(ry);
	(*rotMatrix).at<float>(1,0)=cos(rx)*sin(rz); (*rotMatrix).at<float>(1,1)=cos(rx)*cos(rz); (*rotMatrix).at<float>(1,2)=-sin(rx);
	(*rotMatrix).at<float>(2,0)=cos(ry)*sin(rx)*sin(rz)-cos(rz)*sin(ry); (*rotMatrix).at<float>(2,1)=sin(ry)*sin(rz)+cos(ry)*cos(rz)*sin(rx); (*rotMatrix).at<float>(2,2)=cos(rx)*cos(ry);*/
}

void Model::FilterModel(const cv::Mat& pose,int pointnum)
{
	
	float x = pose.at<float>(0,0); float y = pose.at<float>(0,1); float z = pose.at<float>(0,2);
	float rx = pose.at<float>(0,3); float ry = pose.at<float>(0,4); float rz = pose.at<float>(0,5);
#ifdef MY
	//camera intrinsic
	static cv::Mat intrinsic(3,3,CV_32FC1);
	intrinsic.at<float>(0,0)=m_calibration.fx(); intrinsic.at<float>(0,1)=0; intrinsic.at<float>(0,2)=m_calibration.cx(); //intrinsic.at<float>(0,3)=0;
	intrinsic.at<float>(1,0)=0; intrinsic.at<float>(1,1)=m_calibration.fy(); intrinsic.at<float>(1,2)=m_calibration.cy(); //intrinsic.at<float>(1,3)=0;
	intrinsic.at<float>(2,0)=0; intrinsic.at<float>(2,1)=0; intrinsic.at<float>(2,2)=1; //intrinsic.at<float>(2,3)=0;

	//camera extrinsic

	cv::Mat extrinsic(3,4,CV_32FC1);
	computeExtrinsicByEuler(&extrinsic,x,y,z,rx,ry,rz); //4*4;
#else
	//camera intrinsic
	static cv::Mat intrinsic(3,4,CV_32FC1);
	intrinsic.at<float>(0,0)=m_calibration.fx(); intrinsic.at<float>(0,1)=0; intrinsic.at<float>(0,2)=m_calibration.cx(); intrinsic.at<float>(0,3)=0;
	intrinsic.at<float>(1,0)=0; intrinsic.at<float>(1,1)=m_calibration.fy(); intrinsic.at<float>(1,2)=m_calibration.cy(); intrinsic.at<float>(1,3)=0;
	intrinsic.at<float>(2,0)=0; intrinsic.at<float>(2,1)=0; intrinsic.at<float>(2,2)=1; intrinsic.at<float>(2,3)=0;

	//camera extrinsic

	cv::Mat extrinsic = GetPoseMatrix(); //4*4;
#endif
	//compute the points in the 3d space
	static cv::Mat _pos(4,m_model->numvertices+1,CV_32FC1);
	static cv::Mat _result(3,m_model->numvertices+1,CV_32FC1);
	static cv::Mat _result2(3,m_model->numvertices+1,CV_32FC1);
	for(int i=0; i<=m_model->numvertices; i++)
	{
		_pos.at<float>(0,i) = m_model->vertices[3*(i)+0];
		_pos.at<float>(1,i) = m_model->vertices[3*(i)+1];
		_pos.at<float>(2,i) = m_model->vertices[3*(i)+2];
		_pos.at<float>(3,i) = 1;
	}
	_result = extrinsic*_pos;
	_result2 = intrinsic*_result;  //img location
	int64 time0 = cv::getTickCount();
	//compute the norm of facet
	//float u[3], v[3], n[3], c[3];
//#pragma omp parallel for
	for (int i = 0; i < m_model->numtriangles; i++) {

		float u[3], v[3], n[3], c[3];
		//compute the norm of the triangles
		u[0] = _result.at<float>(0,m_model->triangles[i].vindices[1]) - _result.at<float>(0,m_model->triangles[i].vindices[0]);

		u[1] = _result.at<float>(1,m_model->triangles[i].vindices[1]) - _result.at<float>(1,m_model->triangles[i].vindices[0]);

		u[2] = _result.at<float>(2,m_model->triangles[i].vindices[1]) - _result.at<float>(2,m_model->triangles[i].vindices[0]);

		v[0] = _result.at<float>(0,m_model->triangles[i].vindices[2]) - _result.at<float>(0,m_model->triangles[i].vindices[0]);

		v[1] = _result.at<float>(1,m_model->triangles[i].vindices[2]) - _result.at<float>(1,m_model->triangles[i].vindices[0]);

		v[2] = _result.at<float>(2,m_model->triangles[i].vindices[2]) - _result.at<float>(2,m_model->triangles[i].vindices[0]);

		glmCross(u, v, n);
		glmNormalize(n);

		//center of triangle
		c[0] = 0.25*_result.at<float>(0,m_model->triangles[i].vindices[0]) + 0.25*_result.at<float>(0,m_model->triangles[i].vindices[1]) + 0.5*_result.at<float>(0,m_model->triangles[i].vindices[2]);
		c[1] = 0.25*_result.at<float>(1,m_model->triangles[i].vindices[0]) + 0.25*_result.at<float>(1,m_model->triangles[i].vindices[1]) + 0.5*_result.at<float>(1,m_model->triangles[i].vindices[2]);
		c[2] = 0.25*_result.at<float>(2,m_model->triangles[i].vindices[0]) + 0.25*_result.at<float>(2,m_model->triangles[i].vindices[1]) + 0.5*_result.at<float>(2,m_model->triangles[i].vindices[2]);

		glmNormalize(c);

		//judge the whether the line is visible or not
		float cross = n[0]*c[0] + n[1]*c[1] + n[2]*c[2];
		if(cross < 0.0f)
		{
			if(m_model->lines[m_model->triangles[i].lindices[0]].e1 == 1)
				m_model->lines[m_model->triangles[i].lindices[0]].e2 = 1;
			else
				m_model->lines[m_model->triangles[i].lindices[0]].e1 = 1;

			if(m_model->lines[m_model->triangles[i].lindices[1]].e1 == 1)
				m_model->lines[m_model->triangles[i].lindices[1]].e2 = 1;
			else
				m_model->lines[m_model->triangles[i].lindices[1]].e1 = 1;

			if(m_model->lines[m_model->triangles[i].lindices[2]].e1 == 1)
				m_model->lines[m_model->triangles[i].lindices[2]].e2 = 1;
			else
				m_model->lines[m_model->triangles[i].lindices[2]].e1 = 1;
		}

	}
	
	int64 time1 = cv::getTickCount();
	//printf("model time:%lf\n",(time1-time0)/cv::getTickFrequency());
	//extract the points which can be visible or in the edge of object
	struct _2D3D
	{
		cv::Point3f point_3d;
		cv::Point2d point_2d;
	} _2d3d;

	static std::vector<_2D3D> points_2d_3d(100000);
	int __count = 0;
	static cv::Mat img(m_height,m_width,CV_8UC1,cv::Scalar(0,0,0,0));
	static cv::Mat img1(m_height,m_width,CV_32SC1,cv::Scalar(0,0,0,0));
	static cv::Mat img_contour(m_height,m_width,CV_8UC1,cv::Scalar(0,0,0,0));
	static cv::Mat draw_contour(m_height,m_width,CV_8UC3,cv::Scalar(0,0,0,0));
	for(int r=0; r<m_height; r++)
	{
		for(int c=0; c<m_width; c++)
		{
			img.at<uchar>(r,c) = 0;
			img1.at<int>(r,c) = 0;
			img_contour.at<uchar>(r,c) = 0;
			/*draw_contour.at<cv::Vec3b>(r,c)[0] = 0;
			draw_contour.at<cv::Vec3b>(r,c)[1] = 0;
			draw_contour.at<cv::Vec3b>(r,c)[2] = 0;*/
		}
	}
//#pragma omp parallel for
	for(int i=0; i<m_model->numLines; i++)
	{
		if((m_model->lines[i].e1 == 1 && m_model->lines[i].e2 == 0) || (m_model->lines[i].e1==0 && m_model->lines[i].e2==1) || (m_model->lines[i].e1==1 && m_model->lines[i].e2==1))
		{
			GLuint l0 = m_model->lines[i].vindices[0];
			GLuint l1 = m_model->lines[i].vindices[1];

			float x0 = m_model->vertices[3*l0];
			float y0 = m_model->vertices[3*l0+1];
			float z0 = m_model->vertices[3*l0+2];

			float x1 = m_model->vertices[3*l1];
			float y1 = m_model->vertices[3*l1+1];
			float z1 = m_model->vertices[3*l1+2];

			
			float u0 = _result2.at<float>(0,l0)/_result2.at<float>(2,l0);
			float v0 = _result2.at<float>(1,l0)/_result2.at<float>(2,l0);
			
			
			float u1 = _result2.at<float>(0,l1)/_result2.at<float>(2,l1);
			float v1 = _result2.at<float>(1,l1)/_result2.at<float>(2,l1);
			
//#pragma omp critical
//			{


			_2d3d.point_2d = cv::Point(u0,v0);
			_2d3d.point_3d = cv::Point3f(x0,y0,z0);
			//points_2d_3d.push_back(_2d3d);
			points_2d_3d[2*__count] = _2d3d;

			_2d3d.point_2d = cv::Point(u1,v1);
			_2d3d.point_3d = cv::Point3f(x1,y1,z1);
			//points_2d_3d.push_back(_2d3d);
			points_2d_3d[2*__count+1] = _2d3d;

			__count++;

			if(u0 >=0 && u0<m_width && v0 >=0 && v0<m_height && u1 >=0 && u1<m_width && v1>=0 && v1<m_height)
			{			
				cv::line(img,cv::Point(u0,v0),cv::Point(u1,v1),cv::Scalar(255,255,255),1);
			}
			
		}
		m_model->lines[i].e1 = 0; m_model->lines[i].e2 = 0;
	}
	__count <<= 1;
	/*cv::imshow("contour",img);
	cv::waitKey();*/
	//find contour
	static CvMemStorage* s_storage = cvCreateMemStorage(0);
	CvSeq* contours = NULL;
	IplImage g_image = img;
	cvFindContours(&g_image,s_storage,&contours,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
	
	
	/*g_image = draw_contour;
	cvDrawContours(&g_image,contours,CV_RGB(255,255,255),CV_RGB(255,255,255),0,2);
	cv::imshow("contour",draw_contour);*/
	//cv::imwrite("bunny_contour.png",draw_contour);
	g_image = img_contour;
	cvDrawContours(&g_image,contours,CV_RGB(255,255,255),CV_RGB(255,255,255),0,CV_FILLED);
	m_point_set.m_fill_object = img_contour;
	//cv::imshow("contour1",m_point_set.m_fill_object);
	//for find 3d points by color index
	for(size_t i=0; i<__count; i++)
	{
		if(points_2d_3d[i].point_2d.x>0 && points_2d_3d[i].point_2d.y>0 && points_2d_3d[i].point_2d.x<m_width && points_2d_3d[i].point_2d.y<m_height)
			img1.at<int>(points_2d_3d[i].point_2d) = i + 1;//have problem
	}


	//searching the contour points, save in the _points_3d struct
	std::vector<cv::Point3f> _points_3d;
	std::vector<float> dist_3d;    //d
	std::vector<float> pow_dist_3d;  //d^2
	std::vector<float> cumulate_dist_3d; //cumulate dist
	float sum_dist_3d = 0.0f;
	dist_3d.push_back(0);
	pow_dist_3d.push_back(0);
	cumulate_dist_3d.push_back(0);

	
	for(CvSeq* c=contours; c!=NULL; c=c->v_next)
	{		
		for(int i=0; i<c->total; i++)
		{
			CvPoint* p = (CvPoint*)cvGetSeqElem(c,i);
			int value = img1.at<int>(cv::Point(p->x,p->y));  //index where the 3d points
			if(value > 0)
			{
				_points_3d.push_back(points_2d_3d[value-1].point_3d);

				//computing distance between two 3d points(_points_3d[current]-_points_3d[previous])
				size_t size = _points_3d.size();
				if(size>1)
				{
					float d1 = std::powf(_points_3d[size-1].x-_points_3d[size-2].x,2.0f) + std::powf(_points_3d[size-1].y-_points_3d[size-2].y,2.0f) + std::powf(_points_3d[size-1].z-_points_3d[size-2].z,2.0f);
					float d2 = std::sqrtf(d1);
					dist_3d.push_back(d2);
					pow_dist_3d.push_back(d1);
					sum_dist_3d += d2;
					cumulate_dist_3d.push_back(sum_dist_3d);
				}

			}
					
		}
	}


	/*for(int i=0; i<_points_2d.size(); i++)
	{
		_img.at<uchar>(_points_2d[i]) = 255;

		cv::imshow("_img",_img);
		cv::waitKey();
	}*/


	//put the first point into the last location, get the loop
	if(_points_3d.size() > 1)
		_points_3d.push_back(_points_3d[0]);
	//last dist
	size_t size = _points_3d.size();
	if(size > 1)
	{
		float d1 = std::powf(_points_3d[size-1].x-_points_3d[size-2].x,2.0f) + std::powf(_points_3d[size-1].y-_points_3d[size-2].y,2.0f) + std::powf(_points_3d[size-1].z-_points_3d[size-2].z,2.0f);
		float d2 = std::sqrtf(d1);
		dist_3d.push_back(d2);
		pow_dist_3d.push_back(d1);
		sum_dist_3d += d2;
		cumulate_dist_3d.push_back(sum_dist_3d);
	}

	//sample points, every step is d, a good algorithm below developed by wgf at 2014.10.23
	if(pointnum <= 0)
		pointnum = 100; //in order to not let pointnum = 0
	float d = sum_dist_3d/pointnum;
	if(size > 1)
		m_point_set.m_model_points.push_back(_points_3d[0]);
	//d = 3;
	float cur_sum_dist_3d = d;  //current sum of dist in 3d space
	int num = 0;
	while(cur_sum_dist_3d < sum_dist_3d)  //current sum < whole sum
	{
		num++;
		if(num == pointnum)
			break;
		//judge the point between which two points
		int index1, index2;  //sample point between index1, index2
		for(size_t i=0; i<cumulate_dist_3d.size(); i++)
		{
			if(cur_sum_dist_3d < cumulate_dist_3d[i])
			{
				index1 = i-1;
				index2 = i;
				break;
			}
		}
		cv::Point3f p1 = _points_3d[index1];
		cv::Point3f p2 = _points_3d[index2];
		float lambda = 1 - std::sqrtf(std::powf(cur_sum_dist_3d-cumulate_dist_3d[index1],2.0f)/pow_dist_3d[index2]);
		cv::Point3f p;
		p.x = lambda*p1.x + (1-lambda)*p2.x;
		p.y = lambda*p1.y + (1-lambda)*p2.y;
		p.z = lambda*p1.z + (1-lambda)*p2.z;
		m_point_set.m_model_points.push_back(p);

		cur_sum_dist_3d += d;
		
	}

	//generate the img points of sample points, i indicates img, o indicates obj
	size_t i_size = m_point_set.m_model_points.size();
	cv::Mat o_pos(4,i_size,CV_32FC1);
	cv::Mat i_result(3,i_size,CV_32FC1);

	for(size_t i=0; i<i_size; i++)
	{
		o_pos.at<float>(0,i) = m_point_set.m_model_points[i].x;
		o_pos.at<float>(1,i) = m_point_set.m_model_points[i].y;
		o_pos.at<float>(2,i) = m_point_set.m_model_points[i].z;
		o_pos.at<float>(3,i) = 1;
	}
	i_result = intrinsic*extrinsic*o_pos;
	
	for(int i=0; i<i_size; i++)
	{
		float u1 = i_result.at<float>(0,i)/i_result.at<float>(2,i);
		float v1 = i_result.at<float>(1,i)/i_result.at<float>(2,i);

		m_point_set.m_img_points.push_back(cv::Point2d(u1,v1));

	}


}


void Model::computeExtrinsicByEuler(cv::Mat* mvMatrix, float& _x, float& _y, float& _z, float& _rx, float &_ry, float &_rz)
{
	const float pi = 3.1415926f;
	//float rx = _rx*pi/180; float ry = _ry*pi/180; float rz = _rz*pi/180;
	float rx = _rx; float ry = _ry; float rz = _rz;
	//openglÓëopencv×ø±ê²îÈÆx180¶È
	(*mvMatrix).at<float>(0,0)=cos(ry)*cos(rz); (*mvMatrix).at<float>(0,1)=sin(rx)*sin(ry)-cos(rx)*cos(ry)*sin(rz); (*mvMatrix).at<float>(0,2)=cos(rx)*sin(ry)+cos(ry)*sin(rx)*sin(rz); (*mvMatrix).at<float>(0,3)=_x;
	(*mvMatrix).at<float>(1,0)=sin(rz); (*mvMatrix).at<float>(1,1)=cos(rx)*cos(rz); (*mvMatrix).at<float>(1,2)=-cos(rz)*sin(rx); (*mvMatrix).at<float>(1,3)=_y;
	(*mvMatrix).at<float>(2,0)=-cos(rz)*sin(ry); (*mvMatrix).at<float>(2,1)=cos(ry)*sin(rx)+cos(rx)*sin(ry)*sin(rz); (*mvMatrix).at<float>(2,2)=cos(rx)*cos(ry)-sin(rx)*sin(ry)*sin(rz); (*mvMatrix).at<float>(2,3)=_z;
}

cv::Mat Model::GetPoseMatrix()
{
	cv::Mat rotMat(3,3,CV_32FC1);
	cv::Mat T = cv::Mat::eye(4,4,CV_32FC1);

	cv::Rodrigues(m_rvec,rotMat);
	for(int c=0; c<3; c++)
	{
		for(int r=0; r<3; r++)
		{
			T.at<float>(r,c) = rotMat.at<float>(r,c);
		}
	}
	T.at<float>(0,3)=m_tvec.at<float>(0,0); T.at<float>(1,3)=m_tvec.at<float>(1,0); T.at<float>(2,3) = m_tvec.at<float>(2,0);
	return T;
}

void Model::getIntrinsic(cv::Mat& intrinsic) const
{
	intrinsic = m_calibration.getIntrinsic();

}

void Model::InitPose(const cv::Mat& initPose)
{

	m_rvec.at<float>(0,0) = initPose.at<float>(0,3); m_rvec.at<float>(1,0) = initPose.at<float>(0,4); m_rvec.at<float>(2,0) = initPose.at<float>(0,5);
	m_tvec.at<float>(0,0) = initPose.at<float>(0,0); m_tvec.at<float>(1,0) = initPose.at<float>(0,1); m_tvec.at<float>(2,0) = initPose.at<float>(0,2);
}