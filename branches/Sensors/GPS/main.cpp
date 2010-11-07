/*
 * main.cpp
 *
 *  Created on: Oct 21, 2010
 *  Modified on: Oct 26, 2010 for GPS application
 *      Author: igvcbyu
 */

#include <iostream>

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <mrpt/hwdrivers/CGPSInterface.h>
#include "GpsDevice.h"
using namespace std;


/*
 * 'open_port()' - Open serial port 1.
 *
 * Returns the file descriptor on success or -1 on error.
 */
int open_port() {
	int fd; // File descriptor for the port

	fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);


	 // Could not open the port.

	if (fd == -1) {
		perror("open_port: Unable to open /dev/ttyS0 - ");
	} else {
		fcntl(fd, F_SETFL, 0);
		printf("port opened");
	}
	return (fd);
}

int init_modem(int fd) { /* I - Serial port file */
    char buffer[255];  /* Input buffer */
    char *bufptr;      /* Current char in buffer */
    int  nbytes;       /* Number of bytes read */
    int  tries;        /* Number of tries so far*/

    for (tries = 0; tries < 3; tries ++)
    {
       /* send an AT command followed by a CR*/
	if (write(fd, "AT\r", 3) < 3)
	  continue;

       /* read characters into our string buffer until we get a CR or NL*/
	bufptr = buffer;
	while ((nbytes = read(fd, bufptr, buffer + sizeof(buffer) - bufptr - 1)) > 0)
	{
	  bufptr += nbytes;
	  if (bufptr[-1] == '\n' || bufptr[-1] == '\r')
           break;
	}

       /* nul terminate the string and see if we got an OK response*/
	*bufptr = '\0';

	if (strncmp(buffer, "OK", 2) == 0)
	  return (0);
      }

    return (-1);
    }

void init_port() {
	int fd = open_port();
	struct termios options;

    /*
     * Get the current options for the port...
	 */

    tcgetattr(fd, &options);

    /*
     * Set the baud rates to 19200...
	 */

    cfsetispeed(&options, B19200);
    cfsetospeed(&options, B19200);

    /*
     * Enable the receiver and set local mode...
	 */

    options.c_cflag |= (CLOCAL | CREAD);

    /*
     * Set the new options for the port...
	 */

    tcsetattr(fd, TCSANOW, &options);

    options.c_cflag &= ~CSIZE; /* Mask the character size bits*/
    options.c_cflag |= CS8;    /* Select 8 data bits*/

    /*
     * Set the parity
	 */
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_iflag |= (INPCK | ISTRIP);

    /*
     * Set input mode
	 */
    options.c_lflag |= (ICANON | ECHO | ECHOE);
    /*
     * Set flow control
	 */
    options.c_iflag |= (IXON | IXOFF | IXANY);

    /*
     * Choose output mode
	 */
    options.c_oflag |= OPOST;

    /*
     * set timout for 1 second
     */
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 10;

    int modemWorks = init_modem(fd);
    if (modemWorks){
		printf("\n modem works");
    }
    else printf("modem not working");
}






// Function to be called if ever mrpt decides to begin working...

void gps_comm() {
	const string com = "ttyS0"; // tested on my computer and found this is the port
	int GPSBuffer = 500;
	mrpt::hwdrivers::CGPSInterface gps(GPSBuffer);
	//GpsDevice gps("GPS.ini");

	//cout << com +"before" << endl;
	gps.setSerialPortName ( com ); // pass the name of the serial port to open to mrpt
	//cout << gps.getSerialPortName() << endl;


	gps.doProcess();  // opens, buffers, and tokenizes a gps stream through a port

	if(gps.isGPS_connected())
		printf("Communication established to GPS");
	else if (!gps.isGPS_connected())
		printf("No Communication");


	mrpt::slam::CObservationGPS gpsOUT;
	//cout << gpsOUT.dumpToConsole() << endl;
}


int main() {
	//init_port();
   //printf("Attempting to open port\n");
   //open_port(); // opens a serial port

   //gps_comm(); //establishes communication with the GPS receiver

   return 0;
}
