/*
 *  CameraFeed.h
 *  
 *
 *
 */

#include <iostream>
#include <string>
#include <time.h>
#include "cxcore.h"
#include "cv.h"
#include "highgui.h"
#include "FlyCapture2.h"

using namespace std;
using namespace FlyCapture2;
using namespace cv;

class CameraFeed{

public:
	CameraFeed();
	~CameraFeed();
	int Start();
	int GetFeed();

	void PrintTimeStamp();
	
private:
	// Config information
	FC2Config config;
        BusSpeed isochBusSpeed;
        BusSpeed asyncBusSpeed;
	unsigned int numBuffers;

	// General objects to grab feed
	CameraInfo info;
	Error error;
    	BusManager busMgr;
	PGRGuid guid;
    	Camera cam;
	Image rawImage;
	Image convertedImage;

	// Functions
	void CheckForError(Error error, string message);
	void PrintProperty(Property prop);
	void SetInitialProperties();
	void AdjustShutter(int grayValue, double step, Property * shutterProp);

	// Variables for adjusting shutter
	static const int AVERAGE_FRAME = 10;
	static const int STANDARD_COLOR = 100;
	static const int RANGE = 20;
	static const double MAX_SHUTTER = 4.528;
	
	// Size of ROI
	static const int width = 640;
	static const int height = 10;


	// Property Values
	static const double BRIGHTNESS_VALUE = 0;
	static const double EXPOSURE_VALUE = 1.232;
	static const double SHARPNESS_VALUE = 1532;
	static const double HUE_VALUE = 0;
	static const double SATURATION_VALUE = 124.805;
	static const double GAMMA_VALUE = 1;
	static const double SHUTTER_VALUE = 2.26;
	static const double GAIN_VALUE = 8.551;
	static const double wHITEBALANCE_VALUE_1 = 439;
	static const double wHITEBALANCE_VALUE_2 = 665;


};
