/*
 * DartConstraint.h
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#ifndef SRC_SUBSYSTEMS_DARTCONSTRAINT_H_
#define SRC_SUBSYSTEMS_DARTCONSTRAINT_H_


class DartConstraint {
public:
	const int minimum;
	const int maximum;
	DartConstraint(int min, int max) : minimum(min), maximum(max) {}

	static int mapDartConstraint(int target, std::shared_ptr<DartConstraint> first, std::shared_ptr<DartConstraint> second) {
		double input_range = first->maximum - first->minimum;
		double output_range = second->maximum - second->minimum;
		double output = ((target - first->minimum) / input_range) * output_range + second->minimum;
		return (int) output;
	}

};


#endif /* SRC_SUBSYSTEMS_DARTCONSTRAINT_H_ */
