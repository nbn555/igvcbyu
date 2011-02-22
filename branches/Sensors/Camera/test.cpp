#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "Camera.h"
#include "logging.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

using namespace std;
using namespace cv;




int main(int argc, char *argv[]) {
	Camera camera;
	//camera.loadConfig("test");

	camera.startCamera();
	//camera.getObstacles(NULL);
}

