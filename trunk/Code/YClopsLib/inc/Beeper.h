/**
 * @file Beeper.h
 * @date Feb 6, 2011
 * @author tallred3
 * @brief This is a header file to play the system spkr
 */

#ifndef BEEPER_H_
#define BEEPER_H_

#include <cstdlib>

/**
 * This class is an empty class used only for a grouping of static functions to make the internal pc
 * speaker beep
 */
class Beeper {
public:
	/**
	 * play a beep through the pc_spkr
	 * @param frequency - the frequency of the sound to be played
	 * @param length - the length in milliseconds to play
	 */
	static void beep( int frequency, int length );
private:

	/**
	 * @brief An internal structure to contain data that needs to be passed into the beepi function
	 */
	typedef struct beep_struct {
		int frequency;
		int length;
		struct beep_struct * next;
		explicit beep_struct(int frequency = 0, int length = 0 ):frequency(frequency),length(length), next(NULL){};
	}beep_t;

	static beep_t buffer;

	/**
	 * @brief Internal static method that writes values to the pc speaker
	 * @param void pointer to the location of the beep_struct that contains the sound to play
	 */
	static void * beepi( void * );

	Beeper();
	virtual ~Beeper();

};

#endif /* BEEPER_H_ */
