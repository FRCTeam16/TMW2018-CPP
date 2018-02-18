/*
 * ThresholdCounter.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#include <Util/ThresholdCounter.h>

ThresholdCounter::ThresholdCounter(double _threshold, int _numberOfChecks)
: threshold(_threshold), numberOfChecks(_numberOfChecks) {
}

ThresholdCounter::~ThresholdCounter() {
}

bool ThresholdCounter::Check(double input) {
	if (input > threshold) {
		checkCount++;
	} else {
		Reset();
	}
	return checkCount > numberOfChecks;
}

void ThresholdCounter::Reset() {
	checkCount = 0;
}

