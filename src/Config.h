#ifndef _CONFIG_H
#define _CONFIG_H
#include "glm.h"
#include "CameraCalibration.h"
#include <string>
namespace ols
{
	class Config
	{
	public:
		GLMmodel* model;
		CameraCalibration camCalibration;
		int width;
		int height;
		double fps;
		std::string filename;
	};

}

#endif