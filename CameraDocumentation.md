# Calibrate #
Location:
```
branches/Sensors/Camera/Calibrate
```

This is a python user interface that helps calibrate the camera. It uses pretty much the same code that Y-Clop uses to detect obstacles.  The user interface shows what the camera is seeing and allows you to adjust settings and see how they affect the video feed. This helps when you want to change the threshold and other parameters depending on the environment you are in (indoors, cloudy, sunny). This obviously requires python to be running on the computer. To set up the camera module you need to run the setup.py file in the command line like:

```

python setup.py build
python setup.py install

```

This will compile the "Camera.cpp" file and turn it into a module for python to use. The parameters values for the user interface are loaded from YClops.ini. This is where all the parameters should be saved for the whole project. When you click save on the user interface, the parameter values get saved to the YClops.ini file.

To run the user interface:

```
python simpleGUI.py
```


# Camera #
Location:
```
trunk/Code/YClopsLib/src/Camera.cpp
trunk/Code/YClopsLib/inc/Camera.h
```

Camera.cpp contains the code for finding obstacles. It uses openCV 2.2 libraries so make sure you have those installed. To start out the YClopsLib should only call doProcess and getData. When doProcess is called, the camera gets a frame from the video stream. It then thresholds the image to only show the white in the image. It then distorts the image by key-stoning it so that it is a true representation of the position of the camera. It then splits up the image into a grid. The grid size is determined by the YClops.ini file. It then goes through each cell in the grid and checks to see if an obstacle is in that part of the grid. It does this by measuring how much white is in that cell. If there is too much then it will be marked as containing an obstacle. Each cell corresponds to a different real valued x and y location in front of the robot. It then takes this grid and puts all the values into a map. This map is what is used by YClopsLib.

# Shutter #
Location:
```
branches/Sensors/Camera/Shutter/src/CameraFeed.cpp
branches/Sensors/Camera->Shutter/inc/CameraFeed.h
```

One of the problems that you face when dealing with the camera is that if the environment changes then your obstacle recognition will not work correctly. For example, if you have the parameters set for a sunny day and then clouds cover the sun for a little, this will cause your obstacle recognition to not work properly. To deal with that, we decided to control the shutter speed of the camera since the aperture size is not automatic, but rather manual.

To do this we had to use the FlyCapture software and API that is meant for the camera. This allows you to control pretty much every parameter of the camera you would like.

The Calibrate folder has the packages to install the software on Linux. Talk to Dr. Lee if you want the installer for Windows. There is also documentation for the API in the docs folder.

CameraFeed.cpp first initializes itself by setting the right parameters for the camera. It then connects to the camera and gets an image from it. I then turn this image into a openCV image since that is what I was using for obstacle recognition. This is where you would use the Camera.cpp class (see above) or combine the two classes together. It then creates a region of interest (ROI) at the bottom of the image. It calculates how bright that part of the image is and then adjusts the shutter speed to make the image brighter or darker. The point would be to have some kind of gray square that the camera could see at all times (I programmed it to be the bottom of the image, but it could be anywhere). The camera knows what color this square should be in normal circumstances. It checks the value of it occasionally to make sure it is still that value. If for some reason it is not (it got sunny, or cloudy) then the shutter will change until it gets to that ideal value.