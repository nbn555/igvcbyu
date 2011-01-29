#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "Camera.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

void hasObstacles(int dimension, bool * array, Mat & image);
void distort(Mat & src);
VideoCapture startCamera();
void getFrame(VideoCapture capture, Mat & image);
vector<Mat> convertRGBtoHSV(Mat & image);
void getWhite(Mat & image);
void insertObstacles(/*mrpt::slam::CSimplePointMap & map*/ void * map, int size, bool * array);
void initializeArray(bool * array, int dimension);


void hasObstacles(int dimension, bool * array, Mat & image){
	int height = image.rows / dimension;
	int width = image.cols / dimension;

	int obstacleThreshold = 255 * .3;
	//namedWindow("test");

	for(int i = 0; i < dimension; i++){
		for(int j = 0; j < dimension; j++){
			Mat roi = image(Rect(width * j, height * i, width, height));
			Scalar value = mean(roi);

			//cout << "Mean: " << value[0] << endl;
			if(value[0] > obstacleThreshold){
				array[(i * dimension) + j] = true;
				roi = Scalar(155);

				//imshow("test", image);
				//waitKey();
			}
		}
	}
}

void distort(Mat & src) {
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

	//if(array[row*row_amt+col])
	//{
	//use known row and col position to get pixel, so pixel_x = col*row_amt and pixel_y=row*row_amt

	//}

	Mat warp_matrix = getPerspectiveTransform(srcQuad, dstQuad);

	warpPerspective( src, dst, warp_matrix, src.size() );

	//namedWindow( "Perspective_Warp");
	//imshow( "Perspective_Warp", dst );
	//waitKey();

	src = dst.clone();

}

VideoCapture startCamera(){
	VideoCapture capture(0);

	// check to see if camera was opened
	if(!capture.isOpened()){
		cout << "Error opening camera" << endl;
	}

	return capture;
}

void getFrame(VideoCapture capture, Mat & image){
	// get frame from camera
	capture >> image;
}

/*
 * vector[0] - hue
 * vector[1] - saturation
 * vector[2] - value
 */
vector<Mat> convertRGBtoHSV(Mat & image){
	Mat temp = image.clone();
	cvtColor(temp, temp, CV_RGB2HSV);

	// split multi-channel picture into single-channels
	vector<Mat> vect(image.channels());
	split(temp, vect);

	return vect;
}

void getWhite(Mat & image){
	//namedWindow("prewhite");
	//namedWindow("white");
	//namedWindow("lined white");

	// vect[2] seems to be the best for white
	Mat white = convertRGBtoHSV(image)[1];

	//imshow("prewhite", white.clone());

	////// take out the white color //////
	threshold(white, white, 20, 255, THRESH_BINARY_INV);

	erode(white, white, Mat(), Point(-1, -1), 2);
	dilate(white, white, Mat(), Point(-1, -1), 2);

	//imshow("white", white);

/*
	Mat linedWhite;
	cvtColor( white, linedWhite, CV_GRAY2BGR );

    vector<Vec4i> lines;
    HoughLinesP( white, lines, 1, CV_PI/180, 80, 10, 10 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( linedWhite, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
    }
*/

	//imshow("lined white", linedWhite);

	//waitKey();

	image = white.clone();
}


void insertObstacles(/*mrpt::slam::CSimplePointMap & map*/ void * map, int size, bool * array){
	int pixel_x;
	int pixel_y;
	double x;
	double y;

	for(int i = 0; i < size; i++){
		for(int j = 0; j < size; j++){

			if(array[i * size + j]){
				pixel_x = j * (640/size);
				pixel_y = 480 - (i * (480/size));

				// each pixel is .00635 meters high and wide
				x = ((pixel_x - 240) * .00635);
				y = ((pixel_y * .00635) + .914);

				//map.insertPoints(x, y);

				cout << "x = " << x << endl;
				cout << "y = " << y << endl;
				cout << endl;

			}
		}

	}
}


void initializeArray(bool * array, int dimension){
	for(int i = 0; i < dimension * dimension; i++){
		array[i] = false;
	}
}


int main(int argc, char *argv[]) {
	Camera camera;
	camera.loadConfig("test");
	camera.startCamera();
	camera.getObjstacles(NULL);
/*
	Mat image;
	int const DIMENSION = 30;

	bool * array = new bool[DIMENSION * DIMENSION];
	initializeArray(array,  DIMENSION);

	//VideoCapture capture = startCamera();

	while(true){
		getFrame(capture, image);
		getWhite(image);
		distort(image);

		hasObstacles(DIMENSION, array, image);
		insertObstacles(NULL, DIMENSION, array);

		cout << "DONE INSERTING POINTS INTO MAP" << endl;
	}

	CvCapture * video;
		IplImage * frame;

		// capture the camera
		video = cvCreateCameraCapture(1);

		namedWindow("window");

	    while (true) {
	    	frame = cvQueryFrame(video);
	    	if(!frame) {
				printf("error loading frame.\n");
				return -1;
	    	}

	    	Mat src(frame);

	    	getWhite(src);
	    	//distort(src);
	    	hasObstacles(DIMENSION, array, src);

	    	imshow("window", src);

			// press 'esc' to exit out of program
			char c = waitKey(33);
			if(c == 27) break;

	    }


	delete[] array;
	*/
}

