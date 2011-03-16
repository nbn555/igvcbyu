/**
 * @file Beeper.cpp
 * @date Feb 6, 2011
 * @author tallred3
 * @brief implementation for functions to beep the pcspkr
 */

#include "Beeper.h"
#include <pthread.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/kd.h>

using namespace std;

Beeper::beep_t Beeper::buffer = Beeper::beep_struct(0,0); //!Data structure to play the beeper

void Beeper::beep( int frequency, int length ) {
	pthread_t t;
	beep_t buffer = beep_t(frequency,length);			//!This may be a problem taking the address of a local
	pthread_create( &t, NULL, Beeper::beepi, &buffer );	//!Create a thread to make the system commands, however
														//!I think the ioctl write is non blocking so this may be unnecessary
	pthread_detach( t );

}

void * Beeper::beepi( void * data ) {
	fprintf(stdout, "beep\n");
	//system("beep");
	int fd = open( "/dev/console", O_RDONLY );
	beep_t * thisBeep = ((beep_t*)data);

	int arg = ((thisBeep->length)<<16)+(1193180/(thisBeep->frequency)); //!Don't ask how it works I found it online somewhere

	ioctl(fd, KDMKTONE,arg);
	fprintf(stdout, "beep done\n");
	pthread_exit(NULL);
	return NULL;
}
