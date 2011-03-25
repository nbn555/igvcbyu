/**
 * @file Camera.h
 * @date Jan 27, 2011
 * @author philip lundrigan
 * @brief Camera Sensor Header File
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <pthread.h>
#include <mrpt/slam.h>
#include "YClopsSensor.h"

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>


class Camera : public YClopsSensor {

public:
	Camera();

	/**
	 * Camera - constructor for camera object
	 */
	virtual ~Camera();

	/**
	 * loadConfiguration - loads configuration file that sets up the camera parameters.
	 * Config file should have parameters for percent_filled, grid_size, threshold,
	 * erode_amount, and hsv_vector.
	 *
	 * @param config - the configuration file object
	 * @param sectionName - set to "CAMERA"
	 */
	void loadConfiguration(const mrpt::utils::CConfigFileBase & config, const std::string & sectionName);

	/**
	 * init - sets up the camera object. Checks to make sure the camera is ready.
	 */
	void init();

	/**
	 * sensorProcess - deletes old map and puts new obstacles into map
	 */
	void sensorProcess();

	/**
	 * getData - gets the obstacle map from the camera. Map is wrapped in SensorData object
	 *
	 * @return SensorData object pointer that wraps a obstacle map
	 */
	SensorData * getData();

	/**
	 * getObstacles - used by sensorProcess. It detects obstacles and put them into a map.
	 * Can be used to see what is going on in the camera.
	 *
	 * @param map - a map where the obstacles are put
	 */
	void getObstacles(mrpt::slam::CSimplePointsMap & map);
	void dumpData( std::ostream & out ) const;

private:
	/**
	 * getFrame - gets the next frame from the VideoCapture object.
	 *
	 * @param image - puts frame in image
	 */
	void getFrame(cv::Mat & image);

	/**
	 * insertObstacles - finds the real x and y of the obstacle and puts it into the map
	 *
	 * @param map - the map where the obstacle points will go
	 * @param size - size of the grid and array
	 * @param array - the array of boolean that contain if there is an obstacle at that point
	 */
	void insertObstacles(mrpt::slam::CSimplePointsMap & map, int size, bool * array);

	/**
	 * hasObstacles - Checks to see if a given part of the image has enough white
	 * to be considered an obstacle. If it has an obstacle then it puts true in the
	 * corresponding part of the array.
	 *
	 * @param array - represents if there is an obstacle on the grid. Value will be true
	 * if it contains an obstacle.
	 * @param image - the image that it is checking to see if there are obstacles
	 */
	void hasObstacles(bool * array, cv::Mat & image);

	/**
	 * getWhite - thresholds the image so that only white shows
	 *
	 * @param image - the image you want to get the white of. image is changed to
	 * the thresholded version of itself
	 */
	void getWhite(cv::Mat & image);

	/**
	 * distort - key-stone's the image to remove distortion
	 *
	 * @param src - the image you want to key-stone. Returned through same object
	 */
	void distort(cv::Mat & src);

	/**
	 * initializeArray - sets all the values of the array to false
	 *
	 * @param array - the array you want to set to false
	 */
	void initializeArray(bool * array);

	/**
	 * convertRGBtoHSV - converts the color space from RGB to HSV
	 *
	 * @param image - the image that you want to convert
	 * @return a vector of HSV color space (vector[0] - hue, vector[1] - saturation
	 * vector[2] - value)
	 */
	std::vector<cv::Mat> convertRGBtoHSV(cv::Mat & image);

	cv::VideoCapture capture;				//!<the object that has the video feed
	cv::Mat image;							//!<holds the current image being processed
	bool * array;							//!<represents if there is an obstacle at a certain part of the grid
	mrpt::slam::CSimplePointsMap * map;		//!<points where obstacles are located (x+ is forward, y+ is left)

	double PERCENT_FILLED;					//!<sets how much of a cell in the grid needs to be filled to be an obstacle
	int GRID_SIZE;							//!<how big the grid should be (GRID_SIZE * GRID_SIZE)
	int THRESHOLD;							//!<what the whit threshold should be set at
	int ERODE_AMOUNT;						//!<how many times the image is eroded to remove noise in image
	int HSV_VECTOR;							//!<which vector to pick from the hsv color space (vector[0] - hue, vector[1] - saturation, vector[2] - value)

};


#endif /* CAMERA_H_ */
