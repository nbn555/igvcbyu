#include "Camera.h"
#include "logging.h"
#include <cassert>

//#define DEBUG

#ifdef DEBUG
	#define DEBUG_COMMAND(cmd) cmd
#else
	#define DEBUG_COMMAND(cmd)
#endif


extern bool bCameraData;

using namespace std;
using namespace cv;

Camera::Camera() : capture(0), array(NULL), map(NULL){
}

Camera::~Camera(){
	delete[] array;
	//delete capture;
	delete map;

	LOG_CAMERA(DEBUG3) << "Deleted array and image capture..." << endl;
}

void Camera::loadConfiguration(const mrpt::utils::CConfigFileBase & config, const std::string & sectionName) {
	LOG_CAMERA(DEBUG3) << "Reading config file..." << endl;

	GRID_SIZE = config.read_int(sectionName, "grid_size", 10);
	PERCENT_FILLED = config.read_double(sectionName, "percent_filled", .3);
	THRESHOLD = config.read_int(sectionName, "threshold", 20);
	ERODE_AMOUNT = config.read_int(sectionName, "erode_amount", 2);
	HSV_VECTOR = config.read_int(sectionName, "hsv_vector", 1);

	LOG_CAMERA(DEBUG3) << "Grid size: " << GRID_SIZE << endl;
	LOG_CAMERA(DEBUG3) << "Percent filled: " << PERCENT_FILLED << endl;
	LOG_CAMERA(DEBUG3) << "Threshold: " << THRESHOLD << endl;
	LOG_CAMERA(DEBUG3) << "Erode amount: " << ERODE_AMOUNT << endl;
	LOG_CAMERA(DEBUG3) << "HSV vector: " << HSV_VECTOR << endl;

	if(GRID_SIZE <= 0) 			LOG_CAMERA(FATAL) << "Grid size is negative or zero" << endl;
	if(PERCENT_FILLED <= 0) 	LOG_CAMERA(FATAL) << "Percent filled is negative or 0" << endl;
	if(PERCENT_FILLED > 1.0)	LOG_CAMERA(FATAL) << "Percent filled is greater than 1" << endl;
	if(THRESHOLD < 0) 			LOG_CAMERA(FATAL) << "Threshold is negative, must be between 0 and 255" << endl;
	if(THRESHOLD > 255) 		LOG_CAMERA(FATAL) << "Threshold is too high, must be between 0 and 255" << endl;
	if(ERODE_AMOUNT < 0) 		LOG_CAMERA(FATAL) << "Erode amount must be greater than 0" << endl;
	if(HSV_VECTOR < 0) 			LOG_CAMERA(FATAL) << "hsv vector is negative, must be between 0 and 3" << endl;
	if(HSV_VECTOR > 3) 			LOG_CAMERA(FATAL) << "hsv vector is too high, must be between 0 and 3" << endl;
}

void Camera::init(){
	namedWindow("debug", 1);
	array = new bool[GRID_SIZE * GRID_SIZE];
	map = new mrpt::slam::CSimplePointsMap;
	//capture = new VideoCapture(0);

	// check to see if camera was opened
	if(!capture.isOpened()){
		LOG_CAMERA(FATAL) << "Error opening camera" << endl;
	}

	LOG_CAMERA(DEBUG3) << "Started camera..." << endl;
}

void Camera::sensorProcess() {
	// clear out old map
	if(map != NULL){
		delete map;
		map = new mrpt::slam::CSimplePointsMap;
	}


	// TODO: Talk to Jack about what to do here
	mrpt::poses::CPose3D pose;

	// Putting data in map
	getObstacles(*map, pose);

}

SensorData * Camera::getData() {
	// Wrapping data in CameraData object
	return new CameraData(*map);
}

void Camera::getFrame(Mat & image){
	capture >> image;
}

void Camera::getObstacles(mrpt::slam::CSimplePointsMap & map, mrpt::poses::CPose3D pose){

	initializeArray(array);
	LOG_CAMERA(DEBUG3) << "Initialized array..." << endl;

	getFrame(image);
	LOG_CAMERA(DEBUG3) << "Got frame..." << endl;

	DEBUG_COMMAND(imshow("test", image));
	DEBUG_COMMAND(waitKey());

	getWhite(image);
	LOG_CAMERA(DEBUG3) << "Thresholded to white..." << endl;
	DEBUG_COMMAND(imshow("test", image));
	DEBUG_COMMAND(waitKey());

	distort(image);
	LOG_CAMERA(DEBUG3) << "Distorted image..." << endl;
	DEBUG_COMMAND(imshow("test", image));
	DEBUG_COMMAND(waitKey());

	hasObstacles(array, image);
	LOG_CAMERA(DEBUG3) << "Checked for obstacles..." << endl;

	insertObstacles(map, GRID_SIZE, array, pose);
	LOG_CAMERA(DEBUG3) << "Inserted obstacles into map..." << endl;

}

void Camera::dumpData( std::ostream & out ) const {
	out << "dumping camera data" << endl;

	if(!capture.isOpened()) out << "ERROR" << endl;
	///namedWindow("debug",1);
	//for(;;)
	//{
		//if(image){
			imshow("debug", image);
		//}

	//	if(waitKey(30) >= 0) break;
	//}
}

void Camera::insertObstacles(mrpt::slam::CSimplePointsMap & map, int size, bool * array, mrpt::poses::CPose3D pose ){
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

				double tempx = x;
				double tempy = y;
				double yaw = pose.yaw();

				x = tempy * sin(yaw) + tempx * cos(yaw) + pose.x();
				y = tempx * sin(yaw) + tempy * cos(yaw) + pose.y();

				map.insertPoint(x, y);

				//LOG_CAMERA(DEBUG4) << "\tx = " << x << endl;
				//LOG_CAMERA(DEBUG4) << "\ty = " << y << endl;

			}
		}

	}
}

void Camera::hasObstacles(bool * array, Mat & image){
	int height = image.rows / GRID_SIZE;
	int width = image.cols / GRID_SIZE;


	int obstacleThreshold = 255 * PERCENT_FILLED;
	LOG_CAMERA(DEBUG4) << "\tObstacle threshold: " << obstacleThreshold << endl;
	//DEBUG_COMMAND(namedWindow("test2", 1));

	for(int i = 0; i < GRID_SIZE; i++){
		for(int j = 0; j < GRID_SIZE; j++){
			Mat roi = image(Rect(width * j, height * i, width, height));
			Scalar value = mean(roi);

			LOG_CAMERA(DEBUG4) << "\tMean: " << value[0] << endl;
			if(value[0] > obstacleThreshold){
				array[(i * GRID_SIZE) + j] = true;
				roi = Scalar(155);

				LOG_CAMERA(DEBUG4) << "\t\tFound an obstacle..." << endl;
				DEBUG_COMMAND(imshow("test", image));
				DEBUG_COMMAND(waitKey());

				if(bCameraData){
					imshow("test", image);
				}
			}
		}
	}
}

void Camera::getWhite(Mat & image){
	// vect[2] seems to be the best for white
	Mat white = convertRGBtoHSV(image)[HSV_VECTOR];

	////// take out the white color //////
	threshold(white, white, THRESHOLD, 255, THRESH_BINARY_INV);

	erode(white, white, Mat(), Point(-1, -1), ERODE_AMOUNT);
	dilate(white, white, Mat(), Point(-1, -1), ERODE_AMOUNT);


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

