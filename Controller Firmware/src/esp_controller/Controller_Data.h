#ifndef CONTROLLER_DATA_H
#define CONTROLLER_DATA_H

#include <Arduino.h>

// Struct containing the Controller state information
struct ControllerData {
    int x;
    int y;
    bool button1;
    bool button2;
    bool button3;
};


/// @brief Compare two ControllerData struct to see if they have any differences in the data they contain
/// @param a Struct 'a' to compare against 'b'
/// @param b Struct 'b' to compare against 'a'
/// @return Returns true if no difference is found, else false as the data inside is different 
bool compareControllerData(ControllerData a, ControllerData b){
    if(a.x != b.x || a.y != a.y)
        return false;
    if(a.button1 != b.button1)
        return false;
    if(a.button2 != b.button2)
        return false;
    if(a.button3 != b.button3)
        return false;
    return true;
}

#endif
