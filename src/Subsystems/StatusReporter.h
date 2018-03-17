/*
 * StatusReporter.h
 *
 *  Created on: Mar 5, 2018
 *      Author: jsmith
 */

#ifndef SRC_SUBSYSTEMS_STATUSREPORTER_H_
#define SRC_SUBSYSTEMS_STATUSREPORTER_H_

#include <sstream>
#include "WPILib.h"

class StatusReporter {
public:
	StatusReporter();
	virtual ~StatusReporter() {}
	void Run();
private:
	std::unique_ptr<SerialPort> serial;
	const char delimiter = ',';

	void SendData();
};

#endif /* SRC_SUBSYSTEMS_STATUSREPORTER_H_ */
