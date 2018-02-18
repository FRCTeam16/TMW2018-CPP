/*
 * DoCurlUp.h
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#ifndef SRC_CLIMB_DOCURLUP_H_
#define SRC_CLIMB_DOCURLUP_H_

#include <Climb/StateTransition.h>

class DoCurlUp: public StateTransition {
private:
	bool finished;
public:
	DoCurlUp();
	virtual ~DoCurlUp();

	void Forward() override;
	void Reverse() override;
	bool IsFinished() override;

};

#endif /* SRC_CLIMB_DOCURLUP_H_ */
