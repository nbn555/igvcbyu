/*
 *  CameraFeed.cpp
 *  
 *
 *
 */

#include "CameraFeed.h"

main(){
	CameraFeed cameraFeed;
	cameraFeed.Start();
	cameraFeed.GetFeed();

}

CameraFeed::CameraFeed(){

}

CameraFeed::~CameraFeed(){

}



int CameraFeed::Start(){
	cout << "Starting..." << endl;
    	
        error = busMgr.GetCameraFromIndex(0, &guid);
	CheckForError(error, "could not find camera");

	
	cout << "Setting the parameters..." << endl;
	config.numBuffers = 2;
	config.isochBusSpeed = BUSSPEED_S800;
	config.asyncBusSpeed = BUSSPEED_S800;
	config.grabMode = DROP_FRAMES;
	cam.SetConfiguration(&config);

	cam.SetVideoModeAndFrameRate(VIDEOMODE_640x480RGB, FRAMERATE_30);

	SetInitialProperties();

	cout << "Done setting up the parameters..." << endl;

    	return 0;
}


int CameraFeed::GetFeed(){
	cout << "Starting camera feed..." << endl;    
	
    	// Connect to a camera
    	error = cam.Connect(&guid);
    	CheckForError(error, "Could not connect to Camera");

    	// Start capturing images
    	error = cam.StartCapture();
	CheckForError(error, "Could not start capture on camera");
                
        // Retrieve an image
        error = cam.RetrieveBuffer( &rawImage );
	CheckForError(error, "Could not retrieve buffer from camera");

	// Creating an OpenCV image from FlyCapture image
	Mat image(rawImage.GetRows(),
		 rawImage.GetCols(),
		 CV_8UC3,
		 rawImage.GetData());

	// Create an empty image to store the converted image
	Mat convertedImage(image.size(), CV_8UC3);

	// Taking care of windows to display images
	namedWindow("openCV image (converted)", CV_WINDOW_AUTOSIZE);
	namedWindow("roi", CV_WINDOW_AUTOSIZE);

	// creating ROI from rectangle
	Rect ROI(640-width, 480-height, width, height);
	Mat imageROI = convertedImage(ROI);

	// getting shutter property
	Property shutterProp(SHUTTER);
	cam.GetProperty(&shutterProp);
	shutterProp.autoManualMode = false;
	shutterProp.absControl = true;
	shutterProp.absValue = SHUTTER_VALUE;
	cam.SetProperty(&shutterProp);


	unsigned index = 0;
	unsigned grayValue = 0;
	while(1){
		index++;

		cvtColor(image, convertedImage, CV_RGB2BGR);
		imshow("openCV image (converted)", convertedImage);
		imshow("roi", imageROI);

		// Converts the ROI into gray scale and then sums
		// the average
		Mat temp(imageROI.size(), CV_8UC1);
		cvtColor(imageROI, temp, CV_RGB2GRAY);
		
		grayValue += mean(temp)[0];
		//cout << "Average of ROI:" << mean(temp)[0] << endl;

		// Figuring out the mean of the ROI
		if(index % AVERAGE_FRAME == 0){
			AdjustShutter(grayValue, 1, &shutterProp);
			grayValue = 0;
		}	

		// Retrieve an image
        	error = cam.RetrieveBuffer( &rawImage );
		CheckForError(error, "Could not retrieve buffer from camera");

		char c = cvWaitKey(33);
		if(c == 27) break;
	}

	
          
    	// Stop capturing images
   	error = cam.StopCapture();
	CheckForError(error, "Could not stop capture");
      
    	// Disconnect the camera
    	error = cam.Disconnect();
	CheckForError(error, "Could not disconnect camera");

    	return 0;

}

void CameraFeed::PrintTimeStamp(){
	time_t rawtime;
	struct tm * timeinfo;

  	time ( &rawtime );
  	timeinfo = localtime ( &rawtime );
	cout << asctime(timeinfo) << endl;

}

void CameraFeed::AdjustShutter(int grayValue, double step, Property * shutterProp){
	int average = grayValue / AVERAGE_FRAME;

	

	cout << "Average : " << average << " Shutter: " << shutterProp->absValue << "\t";
	PrintTimeStamp();
	cout << endl;
	// this means the image is too dark
	// we will adjust the shutter to lighten the image
	if(average < (STANDARD_COLOR - RANGE)){
		cout << "Making image brighter..." << endl;
		shutterProp->absValue += step;
		
	}
	// this means the image is too light
	// we will adjust the shutter to darken the image
	else if(average > (STANDARD_COLOR + RANGE)){
		cout << "Making image darker..." << endl;
		shutterProp->absValue -= step;
	}

	//shutterProp.absValue += step;
	//if(shutterProp.absValue > MAX_SHUTTER){
	//	cout << "Shutter speed has reached its max..." << endl;
	//	shutterProp.absValue = MAX_SHUTTER;
	//}

	cam.SetProperty(shutterProp);
}


void CameraFeed::SetInitialProperties(){
	// Declaring all of the camera property objects
	Property brightnessProp(BRIGHTNESS);
	Property exposureProp(AUTO_EXPOSURE);
	Property sharpnessProp(SHARPNESS);
	Property hueProp(HUE);
	Property saturationProp(SATURATION);
	Property gammaProp(GAMMA);
	Property shutterProp(SHUTTER);
	Property gainProp(GAIN);
	Property whiteBalanceProp(WHITE_BALANCE);

	// Now each property will be locked to a certain value
	// to make sure the image is consistent
	
	cam.GetProperty(&brightnessProp);
	brightnessProp.autoManualMode = false;
	brightnessProp.absControl = true;
	brightnessProp.valueA = BRIGHTNESS_VALUE;
	cam.SetProperty(&brightnessProp);

	cout << "Set brightness..." << endl;

	cam.GetProperty(&exposureProp);
	exposureProp.autoManualMode = false;
	exposureProp.absControl = true;
	exposureProp.valueA = EXPOSURE_VALUE;
	cam.SetProperty(&exposureProp);

	cam.GetProperty(&sharpnessProp);
	sharpnessProp.autoManualMode = false;
	sharpnessProp.absControl = true;
	sharpnessProp.valueA = SHARPNESS_VALUE;
	cam.SetProperty(&sharpnessProp);

	cam.GetProperty(&hueProp);
	hueProp.autoManualMode = false;
	hueProp.absControl = true;
	hueProp.valueA = HUE_VALUE;
	cam.SetProperty(&hueProp);

	cam.GetProperty(&saturationProp);
	saturationProp.autoManualMode = false;
	saturationProp.absControl = true;
	saturationProp.valueA = SATURATION_VALUE;
	cam.SetProperty(&saturationProp);
	
	cam.GetProperty(&gammaProp);
	gammaProp.autoManualMode = false;
	gammaProp.absControl = true;
	gammaProp.valueA = GAMMA_VALUE;
	cam.SetProperty(&gammaProp);

	cam.GetProperty(&gainProp);
	gainProp.autoManualMode = false;
	gainProp.absControl = true;
	gainProp.valueA = GAIN_VALUE;
	cam.SetProperty(&gainProp);

	cam.GetProperty(&whiteBalanceProp);
	whiteBalanceProp.autoManualMode = false;
	whiteBalanceProp.absControl = true;
	whiteBalanceProp.valueA = wHITEBALANCE_VALUE_1;
	whiteBalanceProp.valueB = wHITEBALANCE_VALUE_2;
	cam.SetProperty(&whiteBalanceProp);
}

void CameraFeed::CheckForError(Error error, string message){
	error = cam.Connect(&guid);
    	if (error != PGRERROR_OK){
        	cout << message << endl;
        	exit(-1);
    	}  
}

void CameraFeed::PrintProperty(Property prop){
	cout << "\n" << endl;
	//cout << "Property Type: " << prop.type << endl;
	//cout << "present: " << prop.present << endl;
	cout << "absControl: " << prop.absControl << endl;
	//cout << "onePush: " << prop.onePush << endl;
	//cout << "onOff: " << prop.onOff << endl;
	cout << "autoManualMode: " << prop.autoManualMode << endl;
	cout << "valueA: " << prop.valueA << endl;
	//cout << "valueB: " << prop.valueB << endl;
	cout << "absValue: " << prop.absValue << endl;
}
