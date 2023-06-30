#include <Arduino.h>
#include <Bounce2.h>
#include "WiFi.h"
#include "esp_now.h" 

#include "esp_controller/config.h"
#include "esp_controller/Controller_Data.h"
#include "esp_controller/controller_display.h"

#define SERIAL Serial
#define pwmWrite ledcWrite

// ESP-Now configuration and Variables
#define DEVICE_NAME "ESP32_Controller"
uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0xFA, 0xD9, 0x24};
String success;
esp_now_peer_info_t peerInfo;
String received;

Bounce2::Button btnGreen = Bounce2::Button();
Bounce2::Button btnYellow = Bounce2::Button();
Bounce2::Button btnRed = Bounce2::Button();

// assign impossible data to force inital drawing before it's overwritten with valid old data
ControllerData oldData = {MAX_SPEED, 0, 500, 500, false, false, false, false, 0}; 
ControllerData data = {MAX_SPEED, 0, 0, 0, false, false, false, false, 0};

unsigned long previousLoopTime = 0;
unsigned long previousSendTime = 0;
const unsigned long sendInterval = 200;  // Send interval in milliseconds
const unsigned long loopInterval = 10;  // Loop interval in milliseconds


void pwmSetup(int pin, int channel, int frequency, int resolution){
  ledcSetup(channel, frequency, resolution);
  ledcAttachPin(pin, channel);
}

int applyDeadZone(int value, int threshold) {
    if (abs(value) < threshold) {
        return 0;
    } else {
        return value;
    }
}

void readJoyStick(ControllerData *ControllerData, int xPin, int yPin){
  ControllerData->x = analogRead(xPin);
  ControllerData->y = analogRead(yPin);
  // map the values
  ControllerData->x = map(ControllerData->x, 0, 4095, -ControllerData->maxSpeed, ControllerData->maxSpeed);
  ControllerData->y = map(ControllerData->y, 0, 4095, -ControllerData->maxSpeed, ControllerData->maxSpeed);
  // apply the deadzone filtering
  ControllerData->x = applyDeadZone(ControllerData->x, DEADZONE_THRESHOLD);
  ControllerData->y = applyDeadZone(ControllerData->y, DEADZONE_THRESHOLD);
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
  memcpy(&received, incomingData, sizeof(DEVICE_NAME));
  Serial.print("Bytes received: ");
  Serial.println(len);
  
  Serial.print("Received: ");
  Serial.println(received);
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0){
    success = "Delivery Success :)";
    data.failedCount = 0;
    data.connected = true;
  }
  else{
    success = "Delivery Fail :(";
    if (data.failedCount < 3)
    {
      data.failedCount++;
    }else if (data.failedCount >= 3){
      data.connected = false;
    }
  }
}

void setup_esp_now(){
  WiFi.mode(WIFI_MODE_STA);
  Serial.print("MAC Address: ");
  // WiFi.macAddress().toCharArray(data.macAddr, sizeof(data.macAddr));
  // Serial.println(data.macAddr);
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
    updateButtons(&data);
    // read the joystick values
    readJoyStick(&data, JOY_X_PIN, JOY_Y_PIN);

    // Check if it's time to send the data 
    if (currentMillis - previousSendTime >= sendInterval) {
      previousSendTime = currentMillis;

      // FLIP the Joy Y axis value (Mount the Stick wrong)
      data.y *= -1;
      // data.x *= -1;

      // Send the data
      sendData(&data);

      // Print the controller data
      SERIAL.print(data.button1);
      SERIAL.print(",");
      SERIAL.print(data.button2);
      SERIAL.print(",");
      SERIAL.print(data.button3);
      SERIAL.print(",");
      SERIAL.print(data.x);
      SERIAL.print(",");
      SERIAL.print(data.y);
      SERIAL.print(",");
      SERIAL.print(data.failedCount);
      SERIAL.print(",");
      SERIAL.print(data.connected);
      SERIAL.print(",");
      SERIAL.print(data.maxSpeed);
      SERIAL.print(",");
      SERIAL.print(data.curSpeed);
      SERIAL.println();
        
    }

    //check if data has changed before displaying
    if (!compareControllerData(oldData, data)){
      //display it to the controller screen
      // displayData(&data);
      displayGUIData(&data);
    }
  }

  // copy and store the data in an 'old' space ready for next pass
  oldData = data;
  // memcpy(&oldData, data, sizeof(data));
}