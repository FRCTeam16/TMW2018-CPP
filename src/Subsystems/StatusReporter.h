/*
 * StatusReporter.h
 *
 *  Created on: Mar 5, 2018
 *      Author: jsmith
 */

#ifndef SRC_SUBSYSTEMS_STATUSREPORTER_H_
#define SRC_SUBSYSTEMS_STATUSREPORTER_H_

#include "WPILib.h"

class StatusReporter {
public:
	StatusReporter(int deviceAddress = 16);
	virtual ~StatusReporter() {}
	void Run();
private:
	std::unique_ptr<I2C> i2c;

	void SendData();
};

#endif /* SRC_SUBSYSTEMS_STATUSREPORTER_H_ */
