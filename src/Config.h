#ifndef _CONFIG_H
#define _CONFIG_H
#include "CameraCalibration.h"
#include "glm.h"
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