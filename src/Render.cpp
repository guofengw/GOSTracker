#include "stdafx.h"
#include "Render.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
using namespace osa;

int Render::m_width;
int Render::m_height;
CameraCalibration Render::m_calibration;
uchar* Render::m_renderImg;
float* Render::m_buffer_depth;
std::vector<ShapePoseInfo> Render::m_shapePoseInfo;

double Render::eye[3];
double Render::at[3];
int Render::mouse_x = -1;
int Render::mouse_y = -1;
int Render::mouse_button_type = -1;
bool Render::mouse_button_pressed = false;


void ShapePoseInfo::setBoundBox()
{
	assert(m_shape);
	float radius = glmMaxRadius(m_shape);
	int d = std::ceil(radius/2);
	m_bb.push_back(cv::Point3d(-d,-d,-d));
	m_bb.push_back(cv::Point3d(d,-d,-d));
	m_bb.push_back(cv::Point3d(d,d,-d));
	m_bb.push_back(cv::Point3d(-d,d,-d));
	m_bb.push_back(cv::Point3d(-d,-d,d));
	m_bb.push_back(cv::Point3d(d,-d,d));
	m_bb.push_back(cv::Point3d(d,d,d));
	m_bb.push_back(cv::Point3d(-d,d,d));
}

void Render::init(CameraCalibration& calibration, int width, int height, int argc, char** argv)
{
	m_calibration = calibration;
	m_width = width;
	m_height = height;
	m_renderImg = new uchar[width*height*3];
	m_buffer_depth = new float[width*height];
	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_RGBA|GLUT_DEPTH);
	glutInitWindowPosition(0,0);
	glutInitWindowSize(width,height);
	glutCreateWindow("OpenGL");
	glutDisplayFunc(display);
	/*glutMouseFunc(mouse);
	glutMotionFunc( mouseMovement );
	glutKeyboardFunc(keyboard);*/
	
	eye[0] = 0.0; eye[1] = 0.0; eye[2] = 0.0;
	at[0] = 0.0; at[1] = 0.0; at[2] = -50.0;
	

}

void Render::reshape(int width, int height)
{
	GLfloat projMatrix[16];
	m_width = width;
	m_height = height;
	buildProjectionMatrix(m_calibration,projMatrix);
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(projMatrix);
}

void Render::display(void)
{
	glEnable(GL_DEPTH_TEST);
	glDepthRange(0,1);
	glClearColor(0.0,0.0,0.0,1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHT0);
	/*glPolygonMode(GL_FRONT,GL_LINE);
	glPolygonMode(GL_BACK,GL_LINE);*/

	glMatrixMode(GL_MODELVIEW);

	for(int i=0; i<(int)m_shapePoseInfo.size(); i++)
	{
		glLoadIdentity();
		glLoadMatrixf(m_shapePoseInfo[i].mv_matrix);
		if(m_shapePoseInfo[i].m_shape)
			//glmDraw(m_shapePoseInfo[i].m_shape,GLM_NONE);
			glmDraw(m_shapePoseInfo[i].m_shape,GLM_MATERIAL|GLM_SMOOTH);

	}
	glFlush();

}

void Render::display_wgf(void)
{
	glEnable(GL_DEPTH_TEST);
	glDepthRange(0,1);
	glClearColor(0.0,0.0,0.0,1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHT0);
	/*glPolygonMode(GL_FRONT,GL_LINE);
	glPolygonMode(GL_BACK,GL_LINE);*/

	glMatrixMode(GL_MODELVIEW);

	for(int i=0; i<(int)m_shapePoseInfo.size(); i++)
	{
		glLoadIdentity();
		glLoadMatrixf(m_shapePoseInfo[i].mv_matrix);
		if(m_shapePoseInfo[i].m_shape)
			//glmDraw(m_shapePoseInfo[i].m_shape,GLM_NONE);
			glmDraw_wgf(m_shapePoseInfo[i].m_shape,GLM_SMOOTH|GLM_MATERIAL);
	}
	glFlush();
}
//void Render::mouse(int button, int state, int x, int y)
//{
//
//	switch(button)
//	{
//	case GLUT_LEFT_BUTTON:
//		if(state == GLUT_DOWN)
//		{
//			mouse_button_type = button;
//			mouse_button_pressed = true;
//			mouse_x = x;
//			mouse_y = y;
//		}else
//		{
//			mouse_button_type = -1;
//			mouse_button_pressed = false;
//			mouse_x = x;
//			mouse_y = y;
//		}
//		break;
//	case GLUT_MIDDLE_BUTTON:
//		if(state == GLUT_DOWN)
//		{
//			mouse_button_type = button;
//			mouse_button_pressed = true;
//			mouse_x = x;
//			mouse_y = y;
//		}else
//		{
//			mouse_button_type = -1;
//			mouse_button_pressed = false;
//			mouse_x = x;
//			mouse_y = y;
//		}
//		break;
//	case GLUT_RIGHT_BUTTON:
//		if(state == GLUT_DOWN)
//		{
//			mouse_button_type = button;
//			mouse_button_pressed = true;
//			mouse_x = x;
//			mouse_y = y;
//		}else
//		{
//			mouse_button_type = -1;
//			mouse_button_pressed = false;
//			mouse_x = x;
//			mouse_y = y;
//		}
//		break;
//	}
//}
//
//inline void vec3cross (const GLfloat *v1, const GLfloat *v2, GLfloat *v3)
//{
//	v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
//	v3[1] = v1[2]*v2[0] - v1[0]*v2[2];
//	v3[2] = v1[0]*v2[1] - v1[1]*v2[0];
//}
//inline float vec3norm(GLfloat *v)
//{
//	return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
//}
//
//void Render::mouseMovement(int x, int y)
//{
//	const float PI = 3.1415926f;
//
//	//rotate view
//	if(mouse_button_pressed && mouse_button_type == GLUT_LEFT_BUTTON)
//	{
//		GLfloat vec[3] = {eye[0]-at[0], eye[1]-at[1], eye[2]-at[2]};
//		float len = vec3norm(vec);
//		float theta_yaw = atan2(vec[2], vec[0]);
//		float theta_pitch = atan2(vec[1], sqrt(vec[0] * vec[0] + vec[2] * vec[2]));
//		theta_yaw += 1.0*(x-mouse_x)/m_width/len*100;
//		theta_pitch += 1.0*(y-mouse_y)/m_width/len*100;
//
//		if (theta_pitch > 0.4 * PI) theta_pitch = 0.4 * PI;
//		if (theta_pitch < -0.4 * PI) theta_pitch = -0.4 * PI;
//
//		eye[0] = cosf(theta_yaw)*cosf(theta_pitch)*len + at[0];
//		eye[1] = sinf(theta_pitch)*len + at[1];
//		eye[2] = sinf(theta_yaw)*cosf(theta_pitch)*len + at[2];
//
//		std::cout<<eye[0]<<" "<<eye[1]<<" "<<eye[2]<<" "<<at[0]<<" "<<at[1]<<" "<<at[2]<<std::endl;
//	}
//
//	//pan view
//	if (mouse_button_pressed && mouse_button_type == GLUT_MIDDLE_BUTTON) 
//	{
//		GLfloat vec[3] = {eye[0]-at[0], eye[1]-at[1], eye[2]-at[2]};
//		float len = vec3norm(vec);
//		GLfloat eye_up_direction[3] = {0,1,0};
//
//		GLfloat forward[3];
//		GLfloat side[3];
//		GLfloat up[3];
//
//		
//		forward[0] = vec[0]/len;
//		forward[1] = vec[1]/len;
//		forward[2] = vec[2]/len;
//
//		vec3cross(forward,eye_up_direction,side);
//		GLfloat side_length = vec3norm(side);
//		side[0] = side[0]/side_length;
//		side[1] = side[1]/side_length;
//		side[2] = side[2]/side_length;
//
//		vec3cross(side,forward,up);
//
//		eye[0] += side[0]*(x-mouse_x)/m_width*len;
//		eye[1] += side[1]*(x-mouse_x)/m_width*len;
//		eye[2] += side[2]*(x-mouse_x)/m_width*len;
//
//		eye[0] += up[0]*(y-mouse_y)/m_width*len;
//		eye[1] += up[1]*(y-mouse_y)/m_width*len;
//		eye[2] += up[2]*(y-mouse_y)/m_width*len;
//
//		at[0] += side[0]*(x-mouse_x)/m_width*len;
//		at[1] += side[1]*(x-mouse_x)/m_width*len;
//		at[2] += side[2]*(x-mouse_x)/m_width*len;
//
//		at[0] += up[0]*(y-mouse_y)/m_width*len;
//		at[1] += up[1]*(y-mouse_y)/m_width*len;
//		at[2] += up[2]*(y-mouse_y)/m_width*len;
//
//		std::cout<<eye[0]<<" "<<eye[1]<<" "<<eye[2]<<" "<<at[0]<<" "<<at[1]<<" "<<at[2]<<std::endl;
//	}
//
//	//zoom view
//	if (mouse_button_pressed && mouse_button_type == GLUT_RIGHT_BUTTON) {
//		GLfloat vec[3] = {eye[0]-at[0], eye[1]-at[1], eye[2]-at[2]};
//		float len = vec3norm(vec);
//		vec[0] = vec[0]/len;
//		vec[1] = vec[1]/len;
//		vec[2] = vec[2]/len;
//
//		len *= powf(2,1.0*(y-mouse_y)/m_width*10);
//
//		if(len<1) len = 1;
//		if(len>1000) len = 1000;
//
//		eye[0] = vec[0]*len + at[0];
//		eye[1] = vec[1]*len + at[1];
//		eye[2] = vec[2]*len + at[2];
//
//		std::cout<<eye[0]<<" "<<eye[1]<<" "<<eye[2]<<" "<<at[0]<<" "<<at[1]<<" "<<at[2]<<std::endl;
//	}
//
//	mouse_x = x;
//	mouse_y = y;
//
//	glutReshapeFunc(reshape);
//	glutPostRedisplay();
//	glutMainLoopEvent();
//	glutSwapBuffers();
//	glFlush();
//}

void Render::rendering()
{
	/*ShapePoseInfo poseInfo;
	poseInfo.m_shape = model;
	for(int i=0; i<16; i++)
		poseInfo.mv_matrix[i] = mv_matrix[i];
	m_shapePoseInfo.clear();
	m_shapePoseInfo.push_back(poseInfo);*/
	
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutPostRedisplay();
	glutMainLoopEvent();
	
	//int64 time0 = cv::getTickCount();
	//取buffer里面的像素，得到渲染的图像
	//std::memset(m_renderImg,0,m_width*m_height*3);
	//glReadPixels(0,0,m_width,m_height,GL_RGB,GL_UNSIGNED_BYTE,m_renderImg);  //这个太耗时了，怎么办？一次近0.1s
	
	
	//glutSwapBuffers();
	/*int64 time1 = cv::getTickCount();
	cout<<"rendering time:"<<(time1-time0)/cv::getTickFrequency()<<"sec"<<std::endl;*/
}

void Render::rendering_for_vg()
{
	glutDisplayFunc(display_wgf);
	glutReshapeFunc(reshape);
	glutPostRedisplay();
	glutMainLoopEvent();
	//glutSwapBuffers();
}

void Render::keyboard(unsigned char key, int x, int y)
{
	const float PI = 3.1415926f;
	static float rx = 0.0f;
	static float ry = 0.0f;
	static float rz = 0.0f;
	static float longitude = 270.0f;
	static float latitude = 0.0f;
	static float lng = 0.0f;
	static float lat = 0.0f;
	switch(key)
	{
	case 'q':
		glutDestroyWindow(1);
		break;
	case 'a':   //left
		{
			longitude += 7;
			if(longitude > 360)
				longitude = longitude - 360;
			lng = longitude*PI/180.0f;
			int snum = (int)m_shapePoseInfo.size();
			if(snum > 0)
			{
				float r = 0.0f;

				float cx = 0.0f, cy = 0.0f, cz = 0.0f;
				
				for(int i=0; i<snum; i++)
				{
					cx += m_shapePoseInfo[i].mv_matrix[12];
					cy += m_shapePoseInfo[i].mv_matrix[13];
					cz += m_shapePoseInfo[i].mv_matrix[14];
				}
				/*cx += m_calibration.getExtrinsic().at<float>(0,3);
				cy += m_calibration.getExtrinsic().at<float>(1,3);
				cz += m_calibration.getExtrinsic().at<float>(2,3);*/
				cx /= (snum+1); cy /= (snum+1); cz /= (snum+1);
				
				r = std::pow(cx,2)+std::pow(cy,2)+std::pow(cz,2);
				r = std::sqrt(r);

				//cout<<"r:"<<r<<endl;


				float look_x = r*cos(lat)*cos(lng);
				float look_y = r*sin(lat);
				float look_z = -r + r*cos(lat)*sin(lng);

				eye[0] = look_x; eye[1] = look_y; eye[2] = look_z;
				at[0] = m_shapePoseInfo[0].mv_matrix[12]; at[1] = m_shapePoseInfo[0].mv_matrix[13]; at[2] = m_shapePoseInfo[0].mv_matrix[14];
				//at[0] = cx; at[1] = cy; at[2] = cz;
				//at[0] = m_calibration.getExtrinsic().at<float>(0,3); at[1] = m_calibration.getExtrinsic().at<float>(1,3); at[2] = m_calibration.getExtrinsic().at<float>(2,3);
				//cout<<eye[0]<<" "<<eye[1]<<" "<<eye[2]<<" "<<eye[3]<<" "<<at[0]<<" "<<at[1]<<" "<<at[2]<<endl;
			}
		}
		break;
	case 'd':   //right
		{
			longitude -= 7;
			if(longitude < 360)
				longitude = longitude + 360;
			lng = longitude*PI/180.0f;
			int snum = (int)m_shapePoseInfo.size();
			if(snum > 0)
			{
				float r = 0.0f;

				float cx = 0.0f, cy = 0.0f, cz = 0.0f;
				
				for(int i=0; i<snum; i++)
				{
					cx += m_shapePoseInfo[i].mv_matrix[12];
					cy += m_shapePoseInfo[i].mv_matrix[13];
					cz += m_shapePoseInfo[i].mv_matrix[14];
				}
				cx += m_calibration.getExtrinsic().at<float>(0,3);
				cy += m_calibration.getExtrinsic().at<float>(1,3);
				cz += m_calibration.getExtrinsic().at<float>(2,3);
				cx /= (snum+1); cy /= (snum+1); cz /= (snum+1);
				
				r = std::pow(cx,2)+std::pow(cy,2)+std::pow(cz,2);
				r = std::sqrt(r);

				//cout<<"r:"<<r<<endl;


				float look_x = r*cos(lat)*cos(lng);
				float look_y = r*sin(lat);
				float look_z = -r + r*cos(lat)*sin(lng);

				eye[0] = look_x; eye[1] = look_y; eye[2] = look_z;
				at[0] = m_shapePoseInfo[0].mv_matrix[12]; at[1] = m_shapePoseInfo[0].mv_matrix[13]; at[2] = m_shapePoseInfo[0].mv_matrix[14];
				//at[0] = cx; at[1] = cy; at[2] = cz;
				//at[0] = m_calibration.getExtrinsic().at<float>(0,3); at[1] = m_calibration.getExtrinsic().at<float>(1,3); at[2] = m_calibration.getExtrinsic().at<float>(2,3);
				//cout<<eye[0]<<" "<<eye[1]<<" "<<eye[2]<<" "<<eye[3]<<" "<<at[0]<<" "<<at[1]<<" "<<at[2]<<endl;
			}
		}
		break;
	case 'w':   //top
		{
			latitude += 7;
			if(latitude > 90)
				latitude = latitude - 90;
			lat = latitude*PI/180.0f;
			int snum = (int)m_shapePoseInfo.size();
			if(snum > 0)
			{
				float r = 0.0f;

				float cx = 0.0f, cy = 0.0f, cz = 0.0f;
				
				for(int i=0; i<snum; i++)
				{
					cx += m_shapePoseInfo[i].mv_matrix[12];
					cy += m_shapePoseInfo[i].mv_matrix[13];
					cz += m_shapePoseInfo[i].mv_matrix[14];
				}
				cx += m_calibration.getExtrinsic().at<float>(0,3);
				cy += m_calibration.getExtrinsic().at<float>(1,3);
				cz += m_calibration.getExtrinsic().at<float>(2,3);
				cx /= (snum+1); cy /= (snum+1); cz /= (snum+1);
				
				r = std::pow(cx,2)+std::pow(cy,2)+std::pow(cz,2);
				r = std::sqrt(r);


				float look_x = r*cos(lat)*cos(lng);
				float look_y = r*sin(lat);
				float look_z = -r + r*cos(lat)*sin(lng);

				eye[0] = look_x; eye[1] = look_y; eye[2] = look_z;
				at[0] = m_shapePoseInfo[0].mv_matrix[12]; at[1] = m_shapePoseInfo[0].mv_matrix[13]; at[2] = m_shapePoseInfo[0].mv_matrix[14];
				//at[0] = cx; at[1] = cy; at[2] = cz;
				//at[0] = m_calibration.getExtrinsic().at<float>(0,3); at[1] = m_calibration.getExtrinsic().at<float>(1,3); at[2] = m_calibration.getExtrinsic().at<float>(2,3);
				//cout<<eye[0]<<" "<<eye[1]<<" "<<eye[2]<<" "<<eye[3]<<" "<<at[0]<<" "<<at[1]<<" "<<at[2]<<endl;
			}
		}
		break;
	case 's':   //down
		{
			latitude -= 7;
			if(latitude < 0)
				latitude = 90 + latitude;
			lat = latitude*PI/180.0f;
			int snum = (int)m_shapePoseInfo.size();
			if(snum > 0)
			{
				float r = 0.0f;

				float cx = 0.0f, cy = 0.0f, cz = 0.0f;
				
				for(int i=0; i<snum; i++)
				{
					cx += m_shapePoseInfo[i].mv_matrix[12];
					cy += m_shapePoseInfo[i].mv_matrix[13];
					cz += m_shapePoseInfo[i].mv_matrix[14];
				}
				cx += m_calibration.getExtrinsic().at<float>(0,3);
				cy += m_calibration.getExtrinsic().at<float>(1,3);
				cz += m_calibration.getExtrinsic().at<float>(2,3);
				cx /= (snum+1); cy /= (snum+1); cz /= (snum+1);
				
				r = std::pow(cx,2)+std::pow(cy,2)+std::pow(cz,2);
				r = std::sqrt(r);

				//cout<<"r:"<<r<<endl;


				float look_x = r*cos(lat)*cos(lng);
				float look_y = r*sin(lat);
				float look_z = -r + r*cos(lat)*sin(lng);

				eye[0] = look_x; eye[1] = look_y; eye[2] = look_z;
				at[0] = m_shapePoseInfo[0].mv_matrix[12]; at[1] = m_shapePoseInfo[0].mv_matrix[13]; at[2] = m_shapePoseInfo[0].mv_matrix[14];
				//at[0] = cx; at[1] = cy; at[2] = cz;
				//at[0] = m_calibration.getExtrinsic().at<float>(0,3); at[1] = m_calibration.getExtrinsic().at<float>(1,3); at[2] = m_calibration.getExtrinsic().at<float>(2,3);
				//cout<<eye[0]<<" "<<eye[1]<<" "<<eye[2]<<" "<<eye[3]<<" "<<at[0]<<" "<<at[1]<<" "<<at[2]<<endl;
			}
		}
		break;

	}

	glutReshapeFunc(reshape);
	glutPostRedisplay();
	glutMainLoopEvent();
	glutSwapBuffers();
	glFlush();
}

void Render::matrixFromCV2GL(cv::Mat& cv_matrix, GLfloat* gl_matrix)
{
	const float pi = 3.1415926f;
	cv::Mat rx(4,4,CV_32FC1);
	rx.at<float>(0,0)=1; rx.at<float>(0,1)=0; rx.at<float>(0,2)=0; rx.at<float>(0,3)=0;
	rx.at<float>(1,0)=0; rx.at<float>(1,1)=cos(pi); rx.at<float>(1,2)=-sin(pi); rx.at<float>(1,3)=0;
	rx.at<float>(2,0)=0; rx.at<float>(2,1)=sin(pi); rx.at<float>(2,2)=cos(pi); rx.at<float>(2,3)=0;
	rx.at<float>(3,0)=0; rx.at<float>(3,1)=0; rx.at<float>(3,2)=0; rx.at<float>(3,3)=1;

	cv::Mat T = rx*cv_matrix;
	gl_matrix[0] = T.at<float>(0,0); gl_matrix[4] = T.at<float>(0,1); gl_matrix[8] = T.at<float>(0,2); gl_matrix[12] = T.at<float>(0,3);
	gl_matrix[1] = T.at<float>(1,0); gl_matrix[5] = T.at<float>(1,1); gl_matrix[9] = T.at<float>(1,2); gl_matrix[13] = T.at<float>(1,3);
	gl_matrix[2] = T.at<float>(2,0); gl_matrix[6] = T.at<float>(2,1); gl_matrix[10] = T.at<float>(2,2); gl_matrix[14] = T.at<float>(2,3);
	gl_matrix[3] = T.at<float>(3,0); gl_matrix[7] = T.at<float>(3,1); gl_matrix[11] = T.at<float>(3,2); gl_matrix[15] = T.at<float>(3,3);


}

void Render::getDepthImg()
{
	int64 time0 = cv::getTickCount();
	glReadPixels(0,0,m_width,m_height,GL_DEPTH_COMPONENT,GL_FLOAT,m_buffer_depth);

	int64 time1 = cv::getTickCount();
	//printf("read pixel time:%f\n",(time1-time0)/cv::getTickFrequency());
}

void Render::getDepthImg(const cv::Point& p1, const cv::Point& p2)
{
	int width = p2.x - p1.x;
	int height = p2.y - p1.y ;

	float* renderImg = new float[width*height];

	//int64 time0 = cv::getTickCount();
	glReadPixels(p1.x,m_height-p2.y,width,height,GL_DEPTH_COMPONENT,GL_FLOAT,renderImg);  //这个太耗时了，怎么办？一次近0.1s
	//glReadPixels(1,1,202,202,GL_RGB,GL_UNSIGNED_BYTE,renderImg);
	//int64 time1 = cv::getTickCount();
	//printf("read image buffer:%f\n",(time1-time0)/cv::getTickFrequency());
	for(int i=0; i<m_height; i++)
	{
		for(int j=0; j<m_width; j++)
		{
			m_buffer_depth[i*m_width+j] = 1;
		}
	}
	

	for(int i=0; i<width-1; i++)
	{
		for(int j=0; j<height-1; j++)
		{		
			m_buffer_depth[(m_height-p2.y+j)*m_width+p1.x+i] = renderImg[j*width+i];
		}
	}


	delete renderImg;
	
}

cv::Point3f Render::get3DPos(int x, int y)
{

	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	GLfloat winX, winY, winZ;
	GLdouble posX, posY, posZ;
//#pragma omp critical
//	{
	glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
	glGetDoublev( GL_PROJECTION_MATRIX, projection );
	glGetIntegerv( GL_VIEWPORT, viewport );
	
	winX = (float)x;
	winY = (float)viewport[3] - (float)y - 1;
	
	//glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
	winZ = m_buffer_depth[(int)winY*m_width+x];
	
	//printf("wx=%f      wy=%f         wz=%f\n",winX,winY,winZ);
	gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
	//printf("...x=%d  y=%d...px=%.2lf...py=%.2lf...pz=%.2lf\n",x,y,posX,posY,posZ);
//	}
	
	return cv::Point3f(posX,posY,posZ);
	
	
}

cv::Mat Render::getRenderedImg()
{
	glReadPixels(0,0,m_width,m_height,GL_RGB,GL_UNSIGNED_BYTE,m_renderImg);  //这个太耗时了，怎么办？一次近0.1s
	static cv::Mat bufferImg(m_height,m_width,CV_8UC1,cv::Scalar(0,0,0));
	//buffer里面的图像变成opencv里面的图像
	for(int i=0; i<m_width; i++)
	{
		for(int j=0; j<m_height; j++)
		{
			
			bufferImg.at<uchar>(m_height-j-1,i) = m_renderImg[j*m_width*3+3*i];
			//bufferImg.at<uchar>(m_height-j-1,i) = m_renderImg[j*m_width+i];
		}
	}
	
	/*for(int i=0; i<400; i++)
	{
		for(int j=0; j<200; j++)
		{
			bufferImg.at<uchar>(340-j-1,200+i) = m_renderImg[j*400+i];
		}
	}*/
	
	return bufferImg;
}

cv::Mat Render::getRenderedImg(const cv::Point p1, const cv::Point p2)
{
	static cv::Mat bufferImg(m_height,m_width,CV_8UC1,cv::Scalar(0,0,0));
	int width = p2.x - p1.x;
	int height = p2.y - p1.y ;
	
	uchar* renderImg = new uchar[width*height*4];
	
	//int64 time0 = cv::getTickCount();
	glReadPixels(p1.x,m_height-p2.y,width,height,GL_RGBA,GL_UNSIGNED_BYTE,renderImg);  //这个太耗时了，怎么办？一次近0.1s
	//glReadPixels(1,1,202,202,GL_RGB,GL_UNSIGNED_BYTE,renderImg);
	//int64 time1 = cv::getTickCount();
	//printf("read image buffer:%f\n",(time1-time0)/cv::getTickFrequency());
	for(int i=0; i<m_width; i++)
	{
		for(int j=0; j<m_height; j++)
		{
			bufferImg.at<uchar>(j,i) = 0;
		}
	}
	
	for(int i=0; i<width-1; i++)
	{
		for(int j=0; j<height-1; j++)
		{
			bufferImg.at<uchar>(p2.y-j-1,p1.x+i) = renderImg[j*(width)*4+4*i];
			
		}
	}
	

	delete renderImg;
	return bufferImg;
}

void Render::buildProjectionMatrix(const ols::CameraCalibration& calibration, GLfloat* projectionMatrix)
{
	int screen_width = m_width;
	int screen_height = m_height;
	float nearPlane = 0.01f;  // Near clipping distance
	float farPlane  = 1000.0f;  // Far clipping distance


	// Camera parameters
	float f_x = calibration.fx(); // Focal length in x axis
	float f_y = calibration.fy(); // Focal length in y axis (usually the same?)
	float c_x = calibration.cx(); // Camera primary point x
	float c_y = calibration.cy(); // Camera primary point y


	projectionMatrix[0] = 2.0f * f_x / screen_width;
	projectionMatrix[1] = 0.0f;
	projectionMatrix[2] = 0.0f;
	projectionMatrix[3] = 0.0f;


	projectionMatrix[4] = 0.0f;
	projectionMatrix[5] = 2.0f * f_y / screen_height;
	projectionMatrix[6] = 0.0f;
	projectionMatrix[7] = 0.0f;


	projectionMatrix[8] = 2.0f * c_x / screen_width - 1.0f;
	projectionMatrix[9] = 2.0f * c_y / screen_height - 1.0f;    
	projectionMatrix[10] = -( farPlane + nearPlane) / ( farPlane - nearPlane );
	projectionMatrix[11] = -1.0f;


	projectionMatrix[12] = 0.0f;
	projectionMatrix[13] = 0.0f;
	projectionMatrix[14] = -2.0f * farPlane * nearPlane / ( farPlane - nearPlane );        
	projectionMatrix[15] = 0.0f;
}