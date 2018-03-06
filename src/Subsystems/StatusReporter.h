/*
 * LEDCommunications.h
 *
 *  Created on: Mar 2, 2017
 *      Author: User
 */

#ifndef SRC_SUBSYSTEMS_STATUSREPORTER_H_
#define SRC_SUBSYSTEMS_STATUSREPORTER_H_

#include "WPILib.h"
#include "Robot.h"
#include "Robot.cpp"

class StatusReporter {
private:
	std::unique_ptr<I2C> i2c;
	int counter = 0;
	const int scanFrequency = 1;
	void SendData();
public:
	StatusReporter(int portNumber = 16);
	virtual ~StatusReporter() {}
	void Run();
};

#endif /* SRC_SUBSYSTEMS_STATUSREPORTER_H_ */
