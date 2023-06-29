#include "controller_display.h"

//Create a Wire connection object with my I2C pins
TwoWire screenWire(0);
// create display code object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &screenWire, OLED_RESET);

void setupDisplay(){
    //Written with the aid of: https://github.com/adafruit/Adafruit_SSD1306/blob/master/examples/ssd1306_128x64_i2c/ssd1306_128x64_i2c.ino
    screenWire.setPins(22, 23);
    // set the i2c pins for the screen 12c bus connection
    if(!screenWire.begin()){
        Serial.println(F("screenWire Failed to begin"));
    }

    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();
    delay(2000); // Pause for 2 seconds
    // Clear the buffer
    display.clearDisplay();

    //rotate the screen sidewards degrees
    display.setRotation(1);

    //set the display's text parameters
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0,0);

    //print a Test text  
    display.println("This is a Test");
    display.display();
    delay(2000);
    display.clearDisplay();
    display.display();

    Serial.println(F("Display Setup finished"));
}

void displayData(ControllerData* data){
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("X: ");
    display.println(data->x);
    display.print("Y: ");
    display.println(data->x);
    display.display();
}