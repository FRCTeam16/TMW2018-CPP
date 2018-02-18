#include <iostream>
#include <Autonomous/DriveUnit.h>
#include "WPILib.h"

const std::string DriveUnit::PULSES_PER_INCH = "PulsesPerInch";

double DriveUnit::ToPulses(double value, DriveUnit::Units unit) {
	Preferences *prefs = Preferences::GetInstance();
	const double pulsesPerInch = prefs->GetDouble(PULSES_PER_INCH, 1081.95);
	double converted = 0;
	switch (unit) {
	case kInches:
		converted = value * pulsesPerInch;
		break;
	case kPulses:
		converted = value;
		break;
	default:
		std::cerr << "Asked to convert unhandled unit: " << unit << "\n";
		converted = value;
		break;
	}
//	std::cout << "Converted " << value << " to " << converted << " with ppi " << pulsesPerInch << "\n";
	return converted;
}
