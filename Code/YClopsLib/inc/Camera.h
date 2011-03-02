/*
 * Camera.h
 *
 *  Created on: Jan 27, 2011
 *      Author: philiplundrigan
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <pthread.h>
//#include <mrpt/base.h>
#include <mrpt/slam.h>
#include "YClopsSensor.h"

/*
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
*/


#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>


class Camera : public YClopsSensor {

public:
	Camera();

	virtual ~Camera();
	void loadConfiguration(const mrpt::utils::CConfigFileBase & config, const std::string & sectionName);
	void init();
	void sensorProcess();
	SensorData * getData();

	void getObstacles(mrpt::slam::CSimplePointsMap & map, mrpt::poses::CPose3D pose);
	void dumpData( std::ostream & out ) const;

private:
	void getFrame(cv::Mat & image);
	void insertObstacles(mrpt::slam::CSimplePointsMap & map, int size, bool * array, mrpt::poses::CPose3D pose);
	void hasObstacles(bool * array, cv::Mat & image);
	void getWhite(cv::Mat & image);
	void distort(cv::Mat & src);
	void initializeArray(bool * array);
	std::vector<cv::Mat> convertRGBtoHSV(cv::Mat & image);

	static void* cameraDataShow(void*);

	// TODO: Delete me
	cv::VideoCapture capture;
	cv::Mat image;
	bool * array;
	mrpt::slam::CSimplePointsMap * map;

	static bool killThread;
	pthread_t dataDumpThread;
	pthread_attr_t dataDumpThreadAttr;

	double PERCENT_FILLED;
	int GRID_SIZE;
	int THRESHOLD;
	int ERODE_AMOUNT;
	int HSV_VECTOR;

};


#endif /* CAMERA_H_ */
