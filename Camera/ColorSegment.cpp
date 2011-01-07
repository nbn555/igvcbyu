/*
 * ColorSegement.cpp
 *
 *  Created on: Nov 20, 2010
 *      Author: philiplundrigan
 */

#include "ColorSegment.h"
#include <math.h>


int main(int argc, char *argv[]) {
	ColorSegment segment;
	Mat image;


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

    	image = segment.TestThreshold(src);
    	image = segment.detectLines(image);

    	imshow("window", image);

		// press 'esc' to exit out of program
		char c = waitKey(33);
		if(c == 27) break;

    }

    return 0;





	//segment.saveImages();
	//segment.saveRGBtoHSV("image2.jpg");

	//segment.TestRGB("image2.jpg");
	//segment.TestYCrCb("image2.jpg");
	//segment.TestThreshold("outside pictures/image2.jpg");
	//segment.TestThresholdVideo();

}





ColorSegment::ColorSegment(){

}

ColorSegment::~ColorSegment(){

}

Mat ColorSegment::detectLines(Mat & src){
	Mat color_dst;

    //Canny( src, dst, 50, 200, 3 );

	cvtColor( src, color_dst, CV_GRAY2BGR );

    vector<Vec4i> lines;
    HoughLinesP( src, lines, 1, CV_PI/180, 80, 30, 10 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( color_dst, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
    }

    /*
    namedWindow( "Source", 1 );
    imshow( "Source", src );

    namedWindow( "Detected Lines", 1 );
    imshow( "Detected Lines", color_dst );

    waitKey(0);
    */
    return color_dst;
}





void ColorSegment::TestRGB(string imageName){
	Mat image = imread(imageName);


	namedWindow("image");
	namedWindow("red");
	namedWindow("green?");
	namedWindow("blue?");

	vector<Mat> channels(image.channels());
	split(image, channels);

	threshold(channels[0], channels[0], 50, 255, THRESH_BINARY);
	threshold(channels[1], channels[1], 50, 255, THRESH_BINARY);
	threshold(channels[2], channels[2], 100, 255, THRESH_BINARY);

	imshow("image", image);
	imshow("blue?", channels[0]);
	imshow("green?", channels[1]);
	imshow("red", channels[2]);

	waitKey(0);


}

void ColorSegment::TestYCrCb(string imageName){
	Mat image = imread(imageName);


	namedWindow("image");
	namedWindow("channel 1");
	namedWindow("channel 2");
	namedWindow("channel 3");

	vector<Mat> channels = RGBtoYCrCb(image);

	imshow("image", image);
	imshow("channel 1", channels[0]);
	imshow("channel 2", channels[1]);
	imshow("channel 3", channels[2]);

	waitKey(0);


}


Mat ColorSegment::TestThreshold(string imageName){
	Mat image = imread(imageName);
	Mat orange;
	//Mat white;
	//Mat newWhite;
	//Mat bright;
	//Mat yellow;
	imageName = imageName.substr(0, imageName.size() - 4);

	//namedWindow("image");
	//namedWindow("saturation");
	//namedWindow("orange");

	//namedWindow("white");
	//namedWindow("bright");
	//namedWindow("yellow");

	vector<Mat> channels = RGBtoHSV(image);
	orange = getOrange(channels[1]);

	//bitwise_xor(white, bright, yellow);

	//imshow("image", image);
	//imshow("saturation", channels[1]);
	//imshow("orange", orange);

	//imshow("white", white);
	//imshow("bright", bright);
	//imshow("yellow", yellow);

	//waitKey(0);

	return orange;

}

Mat ColorSegment::TestThreshold(Mat & image){
	Mat orange;
	vector<Mat> channels = RGBtoHSV(image);
	orange = getOrange(channels[1]);

	return orange;

}

void ColorSegment::TestThresholdVideo(){
	CvCapture * video;
	IplImage * frame;

	// capture the camera
	video = cvCreateCameraCapture(1);

	Mat orange;

	namedWindow("orange");

    while (true) {
    	frame = cvQueryFrame(video);
    	if(!frame) {
			printf("error loading frame.\n");
			return;
    	}
    	Mat image(frame);

    	vector<Mat> channels = RGBtoHSV(image);
    	orange = getOrange(channels[1]);

    	imshow("orange", orange);
    	//imshow("orange", image);

		// press 'esc' to exit out of program
		char c = waitKey(33);
		if(c == 27) break;

    }

}

Mat ColorSegment::getWhite(Mat & image){
	Mat whiteImage;

	////// take out the orange color //////
	threshold(image, whiteImage, 10, 255, THRESH_BINARY);

	//erode(whiteImage, whiteImage, Mat(), Point(-1, -1), 1);
	//dilate(whiteImage, whiteImage, Mat(), Point(-1, -1), 1);

	return whiteImage;
}

Mat ColorSegment::getOrange(Mat & image){
	Mat orangeImage;

	////// take out the orange color //////
	threshold(image, orangeImage, 150, 255, THRESH_BINARY);

	erode(orangeImage, orangeImage, Mat(), Point(-1, -1), 1);
	dilate(orangeImage, orangeImage, Mat(), Point(-1, -1), 1);

	return orangeImage;
	//return image;
}

/*
 * vector[0] - hue
 * vector[1] - saturation
 * vector[2] - value
 */
vector<Mat> ColorSegment::RGBtoHSV(Mat & image){
	Mat temp = image.clone();
	cvtColor(temp, temp, CV_RGB2HSV);

	// split multi-channel picture into single-channels
	vector<Mat> vect(image.channels());
	split(temp, vect);

	return vect;
}

/*
 * vector[0] - ?
 * vector[1] - ?
 * vector[2] - ?
 */
vector<Mat> ColorSegment::RGBtoYCrCb(Mat & image){
	Mat temp = image.clone();
	cvtColor(temp, temp, CV_RGB2YCrCb);

	// split multi-channel picture into single-channels
	vector<Mat> vect(image.channels());
	split(temp, vect);

	return vect;
}


void ColorSegment::saveRGBtoHSV(string imageName){

	Mat image = imread(imageName);
	imageName = imageName.substr(0, imageName.size() - 4);

	namedWindow("image");
	//namedWindow("converted");
	namedWindow("hue");
	namedWindow("saturation");
	namedWindow("value");

	imshow("image", image);

	// converted from RGB to HSV
	cvtColor(image, image, CV_RGB2HSV);

	//imshow("converted", image);

	// split multi-channel picture into single-channels
	vector<Mat> test(3);
	split(image, test);

	imshow("hue", test[0]);
	imwrite(imageName + "hue.jpg", test[0]);

	imshow("saturation", test[1]);
	imwrite(imageName + "saturation.jpg", test[1]);

	imshow("value", test[2]);
	imwrite(imageName + "value.jpg", test[2]);

	waitKey(0);

}

void ColorSegment::saveImages() {

	CvCapture * video;
	IplImage * frame;
	int imageCounter = 0;

	// capture the camera
    video = cvCreateCameraCapture(1);

    if (!video) {
    	printf("Unable to open camera");
    	return;
    }

    // create an empty window
    cvNamedWindow("window", CV_WINDOW_AUTOSIZE);

    // grab a frame from the video feed
    frame = cvQueryFrame(video);

    while (true) {
    	// show image in window
    	cvShowImage("window", frame);

    	// get new image from video feed
    	frame = cvQueryFrame(video);

    	// check to see if it is a good image
		if(!frame) {
			printf("error loading frame.\n");
			return;
		}

		// press 'esc' to exit out of program
		char c = waitKey(33);
		if(c == 27) break;
		if(c == 's'){
			imageCounter++;
			// Save image
			stringstream stream;
			stream << imageCounter;
			string temp = "image" + stream.str() + ".jpg";
			//string temp = "image2.jpg";
			cvSaveImage(temp.c_str() , frame);
		}
    }

}

