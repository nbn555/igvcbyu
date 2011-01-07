/*
 * ColorSegment.h
 *
 *  Created on: Dec 3, 2010
 *      Author: philiplundrigan
 */

#ifndef COLORSEGMENT_H_
#define COLORSEGMENT_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <highgui.h>
#include <cv.h>
#include <cxcore.h>

using namespace std;
using namespace cv;

class ColorSegment {

public:

	ColorSegment();

	~ColorSegment();

	void TestRGB(string imageName);
	void TestYCrCb(string imageName);
	Mat TestThreshold(string imageName);
	Mat TestThreshold(Mat & image);
	void TestThresholdVideo();
	void saveRGBtoHSV(string imageName);
	void saveImages();

	Mat detectLines(Mat & src);



private:
	vector<Mat> RGBtoHSV(Mat & image);
	vector<Mat> RGBtoYCrCb(Mat & image);

	Mat getOrange(Mat & image);
	Mat getWhite(Mat & image);

};
#endif /* COLORSEGMENT_H_ */
