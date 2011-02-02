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
//#include <mrpt/base.h>
#include <mrpt/slam.h>

/*
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
*/


#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>


using namespace std;
using namespace cv;

class Camera{

public:
	Camera();

	~Camera();

	void loadConfig(String fileName);
	void startCamera();

	void getObstacles(mrpt::slam::CSimplePointsMap & map, mrpt::poses::CPose3D pose);

private:
	void getFrame(Mat & image);
	void insertObstacles(mrpt::slam::CSimplePointsMap & map, int size, bool * array, mrpt::poses::CPose3D pose);
	void hasObstacles(bool * array, Mat & image);
	void getWhite(Mat & image);
	void distort(Mat & src);
	void initializeArray(bool * array);
	vector<Mat> convertRGBtoHSV(Mat & image);

	// TODO: Delete me
	VideoCapture * capture;
	bool * array;

	double PERCENT_FILLED;
	int GRID_SIZE;
	int THRESHOLD;

};


#endif /* CAMERA_H_ */
