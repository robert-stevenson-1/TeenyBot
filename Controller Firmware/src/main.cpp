#include <Arduino.h>
#include <Bounce2.h>
#include "WiFi.h"
#include "esp_now.h" 

#include "esp_controller/config.h"
#include "esp_controller/Controller_Data.h"
#include "esp_controller/controller_display.h"

#include "TeenyBot/Robot_Data.h"

#define SERIAL Serial
#define pwmWrite ledcWrite

// ESP-Now configuration and Variables
#define DEVICE_NAME "ESP32_Controller"
uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0xFA, 0xD9, 0x24};
esp_now_peer_info_t peerInfo;
RobotData receivedRobotData = {255, 0, 0, false, 0};

Bounce2::Button btnGreen = Bounce2::Button();
Bounce2::Button btnYellow = Bounce2::Button();
Bounce2::Button btnRed = Bounce2::Button();

// assign impossible data to force inital drawing before it's overwritten with valid old data
ControllerData oldCntlData = {500, 500, false, false, false, false, 0}; 
ControllerData cntlData = {0, 0, false, false, false, false, 0};

unsigned long previousLoopTime = 0;
unsigned long previousSendTime = 0;
const unsigned long sendInterval = 200;  // Send interval in milliseconds
const unsigned long loopInterval = 10;  // Loop interval in milliseconds


void pwmSetup(int pin, int channel, int frequency, int resolution){
  ledcSetup(channel, frequency, resolution);
  ledcAttachPin(pin, channel);
}

int applyDeadZone(int value, int centerVal, int threshold) {
    if (value > centerVal + threshold || value < centerVal - threshold ) {
        return value;
    } else {
        return centerVal;
    }
}

void readJoyStick(ControllerData *ControllerData, int xPin, int yPin){
  ControllerData->x = analogRead(xPin);
  ControllerData->y = analogRead(yPin);
  // map the values
  // ControllerData->x = map(ControllerData->x, 0, 4095, -255, 255);
  // ControllerData->y = map(ControllerData->y, 0, 4095, -255, 255);
  // apply the deadzone filtering
  ControllerData->x = applyDeadZone(ControllerData->x, JOY_CENTER_VAL, DEADZONE_THRESHOLD);
  ControllerData->y = applyDeadZone(ControllerData->y, JOY_CENTER_VAL , DEADZONE_THRESHOLD);
}

void updateButtons(ControllerData *ControllerData){
  btnRed.update();
  if(btnRed.pressed()){
    ControllerData->button1 = ControllerData->button1 ? false : true; // toggle the value
  }
  btnYellow.update();
  if(btnYellow.pressed()){
    ControllerData->button2 = ControllerData->button2 ? false : true; // toggle the value
  }
  btnGreen.update();
  if(btnGreen.pressed()){
    ControllerData->button3 = ControllerData->button3 ? false : true; // toggle the value
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&receivedRobotData, incomingData, sizeof(receivedRobotData));
  // Serial.print("Bytes received: ");
  // Serial.println(len);  
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0){
    // Delivery Success :)
    cntlData.failedCount = 0;
    cntlData.connected = true;
  }
  else{
    // Delivery Fail :(
    if (cntlData.failedCount < 3)
    {
      cntlData.failedCount++;
    }else if (cntlData.failedCount >= 3){
      cntlData.connected = false;
    }
  }
}

void setup_esp_now(){
  WiFi.mode(WIFI_MODE_STA);
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void sendData(ControllerData* data) {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) data, sizeof((*data)));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void setup() {
  SERIAL.begin(115200);

  //start the screen
  setupDisplay();

  // ESP-Now Initialisation
  setup_esp_now();

  pinMode(JOY_X_PIN, INPUT);
  pinMode(JOY_Y_PIN, INPUT);

  //setup buttons
  btnRed.attach(BTN_RED_PIN, INPUT_PULLUP);
  btnRed.interval(DEBOUNCE_TIME);
  btnRed.setPressedState(LOW);
  
  btnYellow.attach(BTN_YELLOW_PIN, INPUT_PULLUP);
  btnYellow.interval(DEBOUNCE_TIME);
  btnYellow.setPressedState(LOW);

  btnGreen.attach(BTN_GREEN_PIN, INPUT_PULLUP);
  btnGreen.interval(DEBOUNCE_TIME);
  btnGreen.setPressedState(LOW);
}

void loop() {
  unsigned long currentMillis = millis();
  // Main loop code runs every 10 milliseconds
  if (currentMillis - previousLoopTime >= loopInterval) {
    previousLoopTime = currentMillis;

    // Preform Main loop tasks here
    updateButtons(&cntlData);
    // read the joystick values
    readJoyStick(&cntlData, JOY_X_PIN, JOY_Y_PIN);

    // Check if it's time to send the data 
    if (currentMillis - previousSendTime >= sendInterval) {
      previousSendTime = currentMillis;

      // FLIP the Joy Y axis value (Mount the Stick wrong)
      // cntlData.y *= -1;
      // data.x *= -1;

      // Send the data
      sendData(&cntlData);

      // Print the controller data
      SERIAL.print(cntlData.button1);
      SERIAL.print(",");
      SERIAL.print(cntlData.button2);
      SERIAL.print(",");
      SERIAL.print(cntlData.button3);
      SERIAL.print(",");
      SERIAL.print(cntlData.x);
      SERIAL.print(",");
      SERIAL.print(cntlData.y);
      SERIAL.print(",");
      SERIAL.print(cntlData.failedCount);
      SERIAL.print(",");
      SERIAL.print(cntlData.connected);
      SERIAL.print(",");
      SERIAL.print(receivedRobotData.maxSpeed);
      SERIAL.print(",");
      SERIAL.print(receivedRobotData.curSpeedL);
      SERIAL.print(",");
      SERIAL.print(receivedRobotData.curSpeedR);
      SERIAL.println();
        
    }

    //check if data has changed before displaying
    if (!compareControllerData(oldCntlData, cntlData)){
      //display it to the controller screen
      // displayData(&data);
      displayGUIData(&cntlData, &receivedRobotData);
    }
  }

  // copy and store the data in an 'old' space ready for next pass
  oldCntlData = cntlData;
}