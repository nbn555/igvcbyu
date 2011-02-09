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

using namespace std;

Beeper::beep_t Beeper::buffer;

Beeper::Beeper() {
}

Beeper::~Beeper() {
}

void Beeper::beep( int frequency, int length ) {
	pthread_t t;
	buffer = beep_t(frequency,length);
	pthread_create( &t, NULL, Beeper::beepi, &Beeper::buffer );
	pthread_detach( t );

}

void * Beeper::beepi( void * data ) {
	fprintf(stdout, "beep\n");
	system("beep");
	//int fd = open( "/dev/tty0", O_RDONLY );
	beep_t * thisBeep = ((beep_t*)data);
	//int arg = (time<<16)+(1193180/(thisBeep->frequency));
	//ioctl(fd, KDMKTONE,arg);
	//sleep(1);
	fprintf(stdout, "beep done\n");
	pthread_exit(NULL);
	return NULL;
}
