#ifndef ROBOT_DATA_H
#define ROBOT_DATA_H

#include <Arduino.h>

// Struct containing the Robot information
struct RobotData {
    uint8_t maxSpeed;
    uint8_t curSpeedL;
    uint8_t curSpeedR;
    // Connected?
    bool connected;
    uint8_t failedCount;
    // char macAddr[18]; //Eg.0C:B8:15:F8:C2:68
};


/// @brief Compare two RobotData struct to see if they have any differences in the data they contain
/// @param a Struct 'a' to compare against 'b'
/// @param b Struct 'b' to compare against 'a'
/// @return Returns true if no difference is found, else false as the data inside is different 
inline bool compareRobotData(RobotData a, RobotData b){
    if(
        a.maxSpeed != b.maxSpeed ||
        a.curSpeedL != b.curSpeedL ||
        a.curSpeedR != b.curSpeedR ||
        a.connected != b.connected)
        return false;
    return true;
}

#endif
