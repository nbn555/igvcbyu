#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cassert>

#include "Camera.h"
#include "logging.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

using namespace std;
using namespace cv;




int main(int argc, char *argv[]) {
	mrpt::utils::CConfigFile config("Camera.ini");
	Log::SetReportLevel(DEBUG4);
	Log::SetLogFile(&cout);


	Camera camera;

	camera.loadConfig(config, "CAMERA");

	camera.startCamera();

	cout << "Done..." << endl;
	//camera.getObstacles(NULL);

}

