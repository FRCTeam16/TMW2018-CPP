//
// Created by smithj11 on 2/17/2018.
//

#ifndef PROJECT_PREFUTIL_H
#define PROJECT_PREFUTIL_H

#include "WPILib.h"
#include <string>
#include <iostream>

class PrefUtil {
public:
    static double getSet(std::string key, double defaultValue) {
//    	std::cout << "key: " << key << " | value: " << defaultValue << "\n";
        Preferences *prefs = Preferences::GetInstance();
        double returnValue = prefs->GetDouble(key, defaultValue);
        if (!prefs->ContainsKey(key)) {
            prefs->PutDouble(key, returnValue);
        }
//        std::cout << "key: " << key << " | value: " << returnValue << "\n";
        return returnValue;
    }

    static int getSetInt(std::string key, int defaultValue) {
//    	std::cout << "key: " << key << " | value: " << defaultValue << "\n";
		Preferences *prefs = Preferences::GetInstance();
		int returnValue = prefs->GetInt(key, defaultValue);
		if (!prefs->ContainsKey(key)) {
			prefs->PutInt(key, returnValue);
		}
//		std::cout << "key: " << key << " | value: " << returnValue << "\n";
		return returnValue;
	}
};

#endif //PROJECT_PREFUTIL_H
