/*
 * Beeper.h
 *
 *  Created on: Feb 6, 2011
 *      Author: tallred3
 */

#ifndef BEEPER_H_
#define BEEPER_H_

class Beeper {
public:
	Beeper();
	virtual ~Beeper();
	/**
	 * play a beep through the pc_spkr
	 * @param frequency - the frequency of the sound to be played
	 * @param length - the length in milliseconds to play
	 */
	static void beep( int frequency, int length );
private:
	typedef struct beep_struct {
		int frequency;
		int length;
		explicit beep_struct(int frequency = 0, int length = 0 ):frequency(frequency),length(length){};
	}beep_t;
	static void * beepi( void * );
};

#endif /* BEEPER_H_ */
