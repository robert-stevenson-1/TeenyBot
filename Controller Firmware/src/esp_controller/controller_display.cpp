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
    
void displayGUIData(ControllerData* data){
    display.clearDisplay();
    // draw axis illustration arrow
    display.fillTriangle(48, 15, 50, 11, 52, 15, 1);
    display.fillTriangle(54, 11, 56, 15, 58, 11, 1);
    display.fillTriangle(55, 3, 55, 7, 59, 5, 1);
    display.fillTriangle(51, 3, 51, 7, 47, 5, 1);
    // draw the frame
    display.drawRect(0, 0, SCREEN_HEIGHT, 19, 1);
    display.drawRect(0, 18, SCREEN_HEIGHT, 20, 1);
    display.drawRect(0, 37, SCREEN_HEIGHT, SCREEN_WIDTH - 37, 1);

    //set the text size
    display.setTextSize(1);

    // draw the button outline OR fill (it Button toggle True)
    if (data->button1){
        display.setTextColor(BLACK, WHITE);
        display.fillRoundRect(4, 20, 17, 16, 3, 1);
    }else{
        display.setTextColor(WHITE);
        display.drawRoundRect(4, 20, 17, 16, 3, 1);
    }
    display.setCursor(10, 24);
    display.print("1");

    if (data->button2){
        display.setTextColor(BLACK, WHITE);
        display.fillRoundRect(23, 20, 17, 16, 3, 1);
    }else{
        display.setTextColor(WHITE);
        display.drawRoundRect(23, 20, 17, 16, 3, 1);
    }
    display.setCursor(29, 24);
    display.print("2");

    if (data->button3){
        display.setTextColor(BLACK, WHITE);
        display.fillRoundRect(42, 20, 17, 16, 3, 1);
    }else{
        display.setTextColor(WHITE);
        display.drawRoundRect(42, 20, 17, 16, 3, 1);
    }
    display.setCursor(48, 24);
    display.print("3");
    
    // draw the x and y values of the joystick
    display.setTextColor(WHITE);
    display.setCursor(2, 2);
    display.print("X: ");
    display.println(data->x);
    display.setCursor(2, 10);
    display.print("Y: ");
    display.println(data->y);
    display.display();
}

void displayData(ControllerData* data){
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("X: ");
    display.println(data->x);
    display.print("Y: ");
    display.println(data->y);
    display.display();
    display.print("btn1: ");
    display.println(data->button1);
    display.display();
    display.print("btn2: ");
    display.println(data->button2);
    display.display();
    display.print("btn3: ");
    display.println(data->button3);
    display.display();
}