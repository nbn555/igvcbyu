/**
 * @file main.h
 * @date Mar 25, 2011
 * @author tallred3
 * @brief This file shouldn't be included anywhere it is used for documentation purposes only
 */

#ifndef MAIN_H_
#define MAIN_H_
#error "Don't include main.h"

/**
 * @mainpage YClops Documentation
 * @section intro_sec YClops Introduction
 * YClops is an autonomous robot made for the IGVC Competition.  It's sensor inputs are Novatel and PocketMax GPS, Honeywell3000 Compass module, Sick 295 Lidar, Firewire Camera.  For actuators it has 2 one hp motors a Roboteq 2445 Motor controller and two wheel encoders to boot.  Finally as the brain we have a computer, with firewire800 + usb2.0 pci card along with bluetooth and wifi dongles.  As well as A big red button "kill switch" and a wireless panic button
 * @section hardware_sec Hardware Setup
 * @subsection computer_connections Computer Connections
 * The GPS receiver and the compass module both are connected via usb to serial converters while, the motor controller and the lidar are usually plugged into the two serial ports of the mother board.  The lidar is plugged into the serial port because the mother board can convert the rs232 serial port to an rs422 which will allow us to increase the accuracy of the lidar(we currently haven't gotten this to work).
 * @subsection sensor_power Powering the sensors
 * The GPS, Compass, computer, motor controller and kill switch all power themselves from the same unregulated 12V battery.  All of the sensors should have fuses installed in their power lines to protect them from voltage spikes.  The lidar powers itself from two 12V batteries in serial attached in parallel to the other sensors.  The firewire camera powers itself from the regulated 5V line from the computer.
 * @section code_install_sec Software Setup
 * The system is designed to operate on Ubuntu 10.10.  We used Eclipse CDK as the IDE.  You should be able to check out projects from the google code repository at https://igvcbyu.googlecode.com/svn you must be logged in a as a commiter to check out code get the password from Dr. Lee or his hardcopy CD.  Additionally all code and documentation as of the end of the Winter semester 2011 should be available from Dr. Lee in a hard copy CD.
 * @subsection code_projects List of code projects
 * baseMrptProject - a template project which has all the dependencies for compiling against YClopsLib
 * MapCreator - utility to create maps to test in the simulator
 * Simulator - a simulator for the yclops robot used to test the current version of the AI sub system.
 * YClopsLib - The shared library project with all the code.
 */

#endif /* MAIN_H_ */
