# Introduction #

The Goal of the Simulator was to provide an environment where all that is tested is the AI.  We create a simulated robot, which does exactly as told, and it always sees obstacles once it is close enough.  It also always returns exactly where it is and what way it is pointing.


# Maps #

# Loading custom ones #

click the load map button navigate to the map, it only takes the .ym files from our map creator tool.  The robot will start at 0,0 (or where ever it ended on the last map, if this is the second map your doing on this run of the application.

## File Format and creation ##

The maps are a file format from mrpt that we have a tool made to simplify the creation process.  It's the map creator tool on svn.  It's a python program. Run it and it will allow you to open a picture file, most any will do.  than you can specify where you want the center(where the robot will start in simulation) to be than save it.  It will add a .ym extension for you.

### about the pictures ###
This is expecting a gray scale image.  With a consistent white background for no obstacle and black for obstacle.  It works with most common picture formats but we used png most of the time. To make the picture go into paint or something similar. We used gimp.

# Way points File #
## File Format ##
The way points files are text files with pairs of numbers space separated than the pairs separated by newlines.  We have two types, one is in meters stored in the order x,y (what the simulator uses) or degrees lat,long (what we use for comp.)
## Loading ##
Click the load button it will only load ywp files(i think might just be yw). This will load a file and setup the AI to go to those points.  Please note that if the use tsp check box is checked it will do a TSP to determine the shortest route assuming no obstacles other wise it does will navigate them in the order in the file.
# Simulating #
> After the map has been loaded and the way points planned you can hit simulate and it will go, of worth noting is that you don't have to have a way points file but can tell the robot to go somewhere by right clicking and selecting navigate here. Than it will start to run with console output going to the window on the bottom of the screen.
## How it works ##
> The program stores an robot simulator object(instead of the YClopsReactiveNavInterface) that will manipulate the values to do what you tell it to.  than the AI accesses these values using the same function calls as we use when we are actually using sensors and the information is in the same format etc.  Than it tells it to go the same way it tells the motors to.  so from the AI's point of view it has no clue it is telling a simulated robot to move and not the real one.

The configurations for the AI and robot simulator are loaded from .ini files.  We have it defaulting to where we stored ours for simulation.  they are changed from the text boxes, it should be obvious when you have it open. Note that it can store internal ones(which start at hard coded values), but when changed those and used an external one so that we don't have to remake the changes every time we start the application.
