/*
 * FieldInfo.h
 *
 *  Created on: Feb 18, 2018
 *      Author: jsmith
 */

#ifndef SRC_AUTONOMOUS_FIELDINFO_H_
#define SRC_AUTONOMOUS_FIELDINFO_H_

class FieldInfo {

public:
	FieldInfo();
	virtual ~FieldInfo();

	enum Location { Left, Right, Unknown};


	Location switchLocation;
	Location scaleLocation;
	Location farSwitchLocation;
};

#endif /* SRC_AUTONOMOUS_FIELDINFO_H_ */
