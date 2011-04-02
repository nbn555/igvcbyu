/**
 * @file Controller.h
 * @date Mar 26, 2011
 * @author tallred3
 * @brief 
 */

#ifndef YCLOPSCONTROLLER_H_
#define YCLOPSCONTROLLER_H_

#include "AbstractController.h"
#include "YClopsModel.h"

class YClopsController: public MVC::AbstractController {
public:
	YClopsController(util::Observable * m);
	virtual ~YClopsController();
	bool getIsClosing() const;
	virtual void handleEvent();
	void setIsClosing() { isClosing = true; }
	void getRequestedSpeeds( double & linear, double & angular ) { linear = this->linearSpeed; angular = this->angularSpeed; };

protected:
	bool isClosing;
	double linearSpeed;
	double angularSpeed;

	void increaseLogLevel();
	void decreaseLogLevel();

};

#endif /* YCLOPSCONTROLLER_H_ */
