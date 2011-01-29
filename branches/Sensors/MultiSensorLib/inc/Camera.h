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


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


/*
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
*/

using namespace std;
using namespace cv;

class Camera{

public:
	Camera();

	~Camera();

	void loadConfig(String fileName);
	void startCamera();

	void getObstacles(/*mrpt::slam::CSimplePointMap & map*/ void * map);

private:
	void getFrame(Mat & image);
	void insertObstacles(/*mrpt::slam::CSimplePointMap & map*/ void * map, int size, bool * array);
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
