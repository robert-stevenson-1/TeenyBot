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

    display.clearDisplay();
    display.display();

    Serial.println(F("Display Setup finished"));
}
    
void displayGUIData(ControllerData* cntlData, RobotData* robotData){
    display.clearDisplay();
    // draw axis illustration arrow
    display.fillTriangle(48, 15, 50, 11, 52, 15, 1);
    display.fillTriangle(54, 11, 56, 15, 58, 11, 1);
    display.fillTriangle(55, 3, 55, 7, 59, 5, 1);
    display.fillTriangle(51, 3, 51, 7, 47, 5, 1);
    // draw the frames
    display.drawRect(0, 0, SCREEN_HEIGHT, 19, 1);
    display.drawRect(0, 18, SCREEN_HEIGHT, 20, 1);
    display.drawRect(0, 0, SCREEN_HEIGHT, SCREEN_WIDTH, 1);
    display.drawRect(0, 0, SCREEN_HEIGHT, 19, 1);
    display.drawRect(0, SCREEN_WIDTH - 11, SCREEN_HEIGHT, 11, 1);
    display.drawRect(0, 37, SCREEN_HEIGHT, 36, 1); // IMU Data frame

    //set the text size
    display.setTextSize(1);

    // draw the button outline OR fill (it Button toggle True)
    if (cntlData->button1){
        display.setTextColor(BLACK, WHITE);
        display.fillRoundRect(4, 20, 17, 16, 3, 1);
    }else{
        display.setTextColor(WHITE);
        display.drawRoundRect(4, 20, 17, 16, 3, 1);
    }
    display.setCursor(10, 24);
    display.print("1");

    if (cntlData->button2){
        display.setTextColor(BLACK, WHITE);
        display.fillRoundRect(23, 20, 17, 16, 3, 1);
    }else{
        display.setTextColor(WHITE);
        display.drawRoundRect(23, 20, 17, 16, 3, 1);
    }
    display.setCursor(29, 24);
    display.print("2");

    if (cntlData->button3){
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
    display.println(cntlData->x);
    display.setCursor(2, 10);
    display.print("Y: ");
    display.println(cntlData->y);

    // draw the IMU Data (MOCKED RIGHT NOW)
    display.setCursor(2, 40);
    display.print("H: ");
    display.println("359.99");
    display.setCursor(2, 48);
    display.print("X: ");
    display.println("-94.23");
    display.setCursor(2, 56);
    display.print("Y: ");
    display.println("-214.30");
    display.setCursor(2, 64);
    display.print("Z: ");
    display.println("120.56");

    // show if controller is connected to the robot
    display.setCursor(2, SCREEN_WIDTH - 9);
    display.print("Conn: ");
    // display.setCursor(32, SCREEN_WIDTH - 10);
    display.drawRoundRect(32, SCREEN_WIDTH - 9, 30, 7, 2, 1);
    if (cntlData->connected){
        display.fillRoundRect(48, SCREEN_WIDTH - 9, 15, 7, 2, 1);
    }else{
        display.fillRoundRect(32, SCREEN_WIDTH - 9, 15, 7, 2, 1);
    }
    
    // Draw vertical speed bar frames and current levels
    display.drawRoundRect(8, 79, 20, 32, 3, 1); // left bar Frame
    display.drawRoundRect(36, 79, 20, 32, 3, 1); // Right bar frame
    uint8_t fillL = (uint8_t)(map(robotData->curSpeedL, -robotData->maxSpeed, robotData->maxSpeed, 0, 32));
    uint8_t fillR = (uint8_t)(map(robotData->curSpeedR, -robotData->maxSpeed, robotData->maxSpeed, 0, 32));
    // uint8_t fillLPos = (uint8_t)(79 + fillL);
    // uint8_t fillRPos = (uint8_t)(79 + fillR);
    display.fillRoundRect(8, 79, 20, fillL, 3, 1); // left bar frame
    display.fillRoundRect(36, 79, 20, fillR, 3, 1); // Right bar frame



    // display.println(data->connected);
    display.display();
}

void displayData(ControllerData* cntlData, RobotData* robotData){
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("X: ");
    display.println(cntlData->x);
    display.print("Y: ");
    display.println(cntlData->y);
    display.display();
    display.print("btn1: ");
    display.println(cntlData->button1);
    display.display();
    display.print("btn2: ");
    display.println(cntlData->button2);
    display.display();
    display.print("btn3: ");
    display.println(cntlData->button3);
    display.display();
    display.print("Conn: ");
    display.println(cntlData->connected);
    display.display();
    display.print("maxSp: ");
    display.println(robotData->maxSpeed);
    display.display();
    display.print("curSpL: ");
    display.println(robotData->curSpeedL);
    display.display();
    display.print("curSpR: ");
    display.println(robotData->curSpeedR);
    display.display();
}