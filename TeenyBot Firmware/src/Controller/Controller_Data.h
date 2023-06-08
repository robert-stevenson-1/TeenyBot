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

#endif
