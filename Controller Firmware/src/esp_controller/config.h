#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Controller Config
#define NUM_OF_BTN 3
#define DEBOUNCE_TIME 20
#define DEADZONE_THRESHOLD 30 
#define MAX_SPEED 255

// PIN Config
#define BTN_RED_PIN 12
#define BTN_YELLOW_PIN 14
#define BTN_GREEN_PIN 27

#define JOY_X_PIN 33
#define JOY_Y_PIN 32



#endif