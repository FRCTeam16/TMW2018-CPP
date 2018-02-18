/*
 * NoOp.h
 *
 *  Created on: Feb 9, 2017
 *      Author: User
 */

#ifndef SRC_AUTONOMOUS_NOOP_H_
#define SRC_AUTONOMOUS_NOOP_H_
#include <iostream>
#include "../Step.h"


class NoOp :public Step {
public:
	NoOp();
	virtual ~NoOp();
	bool Run(std::shared_ptr<World> world) {
		std::cout << "in NoOp Run\n";
		return false;
	}
};

#endif /* SRC_AUTONOMOUS_NOOP_H_ */
