#include <Python.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace std;
using namespace cv;

void getObstacles(cv::Mat & image);
void hasObstacles(cv::Mat & image);
void getWhite(cv::Mat & image);
void distort(cv::Mat & src);
std::vector<cv::Mat> convertRGBtoHSV(cv::Mat & image);

int GRID_SIZE;
int THRESHOLD;
float PERCENT_FILLED;
int ERODE_AMOUNT;
int HSV_VECTOR;
int DISPLAY_BOXES;
int DISTORT;

VideoCapture capture(0);

static PyObject * update(PyObject *self, PyObject *args){
	
	Mat image;
	namedWindow("test", 1);

	// check to see if camera was opened
	if(!capture.isOpened()){
		cout << "Error opening camera" << endl;
	}
	
	GRID_SIZE = -1;
	THRESHOLD = -1;
	PERCENT_FILLED = -1;
	ERODE_AMOUNT = -1;
	HSV_VECTOR = -1;
	DISPLAY_BOXES = -1;
	DISTORT = -1;
	
	if (!PyArg_ParseTuple(args, "iifiiii", &GRID_SIZE, &THRESHOLD, &PERCENT_FILLED,
						 &ERODE_AMOUNT, &HSV_VECTOR, &DISPLAY_BOXES, &DISTORT)){
    	return NULL;
    }
    
    if(GRID_SIZE == -1 || THRESHOLD == -1 || PERCENT_FILLED == -1 || ERODE_AMOUNT == -1 ||
    	HSV_VECTOR == -1 || DISPLAY_BOXES == -1 || DISTORT == -1){
    	
    	cout << "Error, parameters not set correctly" << endl;
    	return Py_BuildValue("");
    }
    
    // Do camera magic here    
    capture >> image;
    
    if(image.data == NULL){
    	cout << "Image data is empty..." << endl;
    	return Py_BuildValue("");
    }
    
    getObstacles(image);
	imshow("test", image);	
		
	return Py_BuildValue("");
	
}

PyMethodDef methods[] = {
    {"update", update, METH_VARARGS, "updates camera parameters"},
    {NULL, NULL, 0, NULL}
};


PyMODINIT_FUNC initcamera()
{
    (void) Py_InitModule("camera", methods);   
}

/* CAMERA CODE */

void getObstacles(Mat & image){

	getWhite(image);
	
	if(DISTORT){
		distort(image);	
	}
	if(DISPLAY_BOXES){
		hasObstacles(image);
	}

}


void hasObstacles(Mat & image){
	int height = image.rows / GRID_SIZE;
	int width = image.cols / GRID_SIZE;

	int obstacleThreshold = 255 * PERCENT_FILLED;

	for(int i = 0; i < GRID_SIZE; i++){
		for(int j = 0; j < GRID_SIZE; j++){
			Mat roi = image(Rect(width * j, height * i, width, height));
			Scalar value = mean(roi);

			/* SHOW BOXES */
			if(value[0] > obstacleThreshold){
				roi = Scalar(155);

			}
		}
	}
}

void getWhite(Mat & image){
	// vect[2] seems to be the best for white
	Mat white = convertRGBtoHSV(image)[HSV_VECTOR];

	////// take out the white color //////
	threshold(white, white, THRESHOLD, 255, THRESH_BINARY_INV);

	erode(white, white, Mat(), Point(-1, -1), ERODE_AMOUNT);
	dilate(white, white, Mat(), Point(-1, -1), ERODE_AMOUNT);

	image = white.clone();
}

void distort(Mat & src){
	Point2f srcQuad[4], dstQuad[4];
	Mat dst;

	dst = Mat::zeros(src.size(), src.type());

	// rows == height
	// cols == width

	srcQuad[0].x = 0;
	srcQuad[0].y = 0;
	srcQuad[1].x = src.cols - 1;
	srcQuad[1].y = 0;
	srcQuad[2].x = 0;
	srcQuad[2].y = src.rows - 1;
	srcQuad[3].x = src.cols - 1;
	srcQuad[3].y = src.rows - 1;


	dstQuad[0].x = 97; //dst Top Left
	dstQuad[0].y = 0;
	dstQuad[1].x = 543; //dst Top Right
	dstQuad[1].y = 0;
	dstQuad[2].x = 214; //dst Bottom Left
	dstQuad[2].y = src.rows-1;
	dstQuad[3].x = 426; //dst  Bottom Right
	dstQuad[3].y = src.rows-1;

	Mat warp_matrix = getPerspectiveTransform(srcQuad, dstQuad);

	warpPerspective( src, dst, warp_matrix, src.size() );

	src = dst.clone();

}

vector<Mat> convertRGBtoHSV(Mat & image){
	Mat temp = image.clone();
	cvtColor(temp, temp, CV_RGB2HSV);

	// split multi-channel picture into single-channels
	vector<Mat> vect(image.channels());
	split(temp, vect);

	return vect;
}