#include "CrabSpeed.h"

CrabSpeed::CrabSpeed() :
output(0)
{
	
}

CrabSpeed::~CrabSpeed() {

}

void CrabSpeed::PIDWrite(double _output)
{
	output = _output;
}

double CrabSpeed::Get() const
{
	return output;
}
