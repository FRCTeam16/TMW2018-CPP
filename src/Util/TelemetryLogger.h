#ifndef SRC_UTIL_TELEMETRYLOGGER_H_
#define SRC_UTIL_TELEMETRYLOGGER_H_

#include "WPILib.h"
#include <fstream>

class TelemetryLogger {
public:
	TelemetryLogger();
	virtual ~TelemetryLogger();
	void Run();
	void Begin();
	void End();
private:
	bool running = false;
	std::ofstream logFile;
	void Log();
};

#endif /* SRC_UTIL_TELEMETRYLOGGER_H_ */
