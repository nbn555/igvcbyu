/*
 * Beeper.cpp
 *
 *  Created on: Feb 6, 2011
 *      Author: tallred3
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

Beeper::beep_t Beeper::buffer = Beeper::beep_struct(0,0);

Beeper::Beeper() {
}

Beeper::~Beeper() {
}

void Beeper::beep( int frequency, int length ) {
	pthread_t t;
	beep_t buffer = beep_t(frequency,length);//This may be a problem taking the address of a local
	pthread_create( &t, NULL, Beeper::beepi, &buffer );
	pthread_detach( t );

}

void * Beeper::beepi( void * data ) {
	fprintf(stdout, "beep\n");
	//system("beep");
	int fd = open( "/dev/console", O_RDONLY );
	beep_t * thisBeep = ((beep_t*)data);
	int arg = ((thisBeep->length)<<16)+(1193180/(thisBeep->frequency));
	ioctl(fd, KDMKTONE,arg);
	fprintf(stdout, "beep done\n");
	pthread_exit(NULL);
	return NULL;
}
