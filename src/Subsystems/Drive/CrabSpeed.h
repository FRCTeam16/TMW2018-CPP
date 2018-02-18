#ifndef CRABSPEED_H_
#define CRABSPEED_H_

#include "WPILib.h"

class CrabSpeed : public PIDOutput
{
public:
	CrabSpeed();
	virtual ~CrabSpeed();
	virtual void PIDWrite(double _output) override;
	double Get() const;
	
private:
	double output;
};

#endif
