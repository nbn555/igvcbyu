#include "Camera.h"

#define DEBUG_CONTROL_WINDOW
#define DEBUG_CONTROL_CONSOLE

#ifdef DEBUG_CONTROL_CONSOLE
	#define DEBUG(str) cout << str << endl;
#else
	#define DEBUG(str)
#endif


#ifdef DEBUG_CONTROL_WINDOW
	#define DEBUG_COMMAND(command) command
#else
	#define DEBUG_COMMAND(command)
#endif

Camera::Camera(){
	GRID_SIZE = 10;
	PERCENT_FILLED = .3;
	THRESHOLD = 20;

	array = new bool[GRID_SIZE * GRID_SIZE];

}

Camera::~Camera(){
	delete[] array;
	delete capture;

	DEBUG("Deleted array and video capture...");
}

void Camera::loadConfig(String fileName){
	DEBUG("Loaded config file...");
}

void Camera::startCamera(){
	capture = new VideoCapture(0);

	// check to see if camera was opened
	if(!capture->isOpened()){
		cout << "Error opening camera" << endl;
	}

	DEBUG("Started camera...");
}

void Camera::getFrame(Mat & image){
	*capture >> image;
}

void Camera::getObstacles(/*mrpt::slam::CSimplePointMap & map*/ void * map){
	Mat image;
	DEBUG_COMMAND(namedWindow("test", 1));

	initializeArray(array);
	DEBUG("Initialized array...");

	getFrame(image);
	DEBUG("Got frame...");

	DEBUG_COMMAND(imshow("test", image));
	DEBUG_COMMAND(waitKey());

	getWhite(image);
	DEBUG("Thresholded to white...");
	DEBUG_COMMAND(imshow("test", image));
	DEBUG_COMMAND(waitKey());

	distort(image);
	DEBUG("Distorted image...");
	DEBUG_COMMAND(imshow("test", image));
	DEBUG_COMMAND(waitKey());

	hasObstacles(array, image);
	DEBUG("Checked for obstacles...");

	insertObstacles(NULL, GRID_SIZE, array);
	DEBUG("Inserted obstacles into map...");

}

void Camera::insertObstacles(/*mrpt::slam::CSimplePointMap & map*/ void * map, int size, bool * array){
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

				DEBUG("\tx = " << x);
				DEBUG("\ty = " << y << "\n");

			}
		}

	}
}

void Camera::hasObstacles(bool * array, Mat & image){
	int height = image.rows / GRID_SIZE;
	int width = image.cols / GRID_SIZE;


	int obstacleThreshold = 255 * PERCENT_FILLED;
	DEBUG("\tObstacle threshold: " << obstacleThreshold);
	DEBUG_COMMAND(namedWindow("test"));

	for(int i = 0; i < GRID_SIZE; i++){
		for(int j = 0; j < GRID_SIZE; j++){
			Mat roi = image(Rect(width * j, height * i, width, height));
			Scalar value = mean(roi);

			DEBUG("\tMean: " << value[0]);
			if(value[0] > obstacleThreshold){
				array[(i * GRID_SIZE) + j] = true;
				roi = Scalar(155);

				DEBUG("\t\tFound an obstacle...");
				DEBUG_COMMAND(imshow("test", image));
				DEBUG_COMMAND(waitKey());
			}
		}
	}
}

void Camera::getWhite(Mat & image){
	// vect[2] seems to be the best for white
	Mat white = convertRGBtoHSV(image)[1];

	////// take out the white color //////
	threshold(white, white, THRESHOLD, 255, THRESH_BINARY_INV);

	erode(white, white, Mat(), Point(-1, -1), 2);
	dilate(white, white, Mat(), Point(-1, -1), 2);


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

	image = white.clone();
}

void Camera::distort(Mat & src){
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

void inline Camera::initializeArray(bool * array){
	for(int i = 0; i < GRID_SIZE * GRID_SIZE; i++){
		array[i] = false;
	}
}

vector<Mat> Camera::convertRGBtoHSV(Mat & image){
	Mat temp = image.clone();
	cvtColor(temp, temp, CV_RGB2HSV);

	// split multi-channel picture into single-channels
	vector<Mat> vect(image.channels());
	split(temp, vect);

	return vect;
}

