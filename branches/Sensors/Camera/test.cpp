#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "Camera.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;




int main(int argc, char *argv[]) {
	cout << "Starting main method...";
	Camera camera;
	camera.loadConfig("test");

	//camera.startCamera();
	//camera.getObstacles(NULL);
}

