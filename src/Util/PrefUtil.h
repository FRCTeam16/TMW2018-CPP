//
// Created by smithj11 on 2/17/2018.
//

#ifndef PROJECT_PREFUTIL_H
#define PROJECT_PREFUTIL_H

#include "WPILib.h"
#include <string>

class PrefUtil {
public:
    static double getSet(std::string key, double defaultValue) {
        Preferences *prefs = Preferences::GetInstance();
        double returnValue = prefs->GetDouble(key, defaultValue);
        if (!prefs->ContainsKey(key)) {
            prefs->PutDouble(key, returnValue);
        }
        return returnValue;
    }

    static int getSetInt(std::string key, int defaultValue) {
            Preferences *prefs = Preferences::GetInstance();
            int returnValue = prefs->GetInt(key, defaultValue);
            if (!prefs->ContainsKey(key)) {
                prefs->PutInt(key, returnValue);
            }
            return returnValue;
        }
};

#endif //PROJECT_PREFUTIL_H
