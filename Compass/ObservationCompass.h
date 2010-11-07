/*
 * ObservationCompass.h
 *
 *  Created on: Nov 7, 2010
 *      Author: tallred3
 */

#ifndef OBSERVATIONCOMPASS_H_
#define OBSERVATIONCOMPASS_H_

#include <mrpt/slam/CObservation.h>

class ObservationCompass: public mrpt::slam::CObservation {
public:
	ObservationCompass();
	virtual ~ObservationCompass();
};

#endif /* OBSERVATIONCOMPASS_H_ */
