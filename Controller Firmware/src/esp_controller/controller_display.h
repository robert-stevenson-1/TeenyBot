#ifndef CONTROLLER_DISPLAY_H
#define CONTROLLER_DISPLAY_H
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Controller_Data.h"

#define SCREEN_ADDRESS 0x3C // Check Board OR datasheet

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

extern TwoWire screenWire;
extern Adafruit_SSD1306 display;

void setupDisplay();
void displayGUIData(ControllerData* data);
void displayData(ControllerData* data);

#endif