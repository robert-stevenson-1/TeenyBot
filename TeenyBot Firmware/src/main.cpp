#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#include "TeenyBot/Config.h"
#include "TeenyBot/Robot_Data.h"
#include "Controller/Controller_Data.h"

#define SERIAL Serial
#define pwmWrite ledcWrite

// Create the Controller's multiple tasks
TaskHandle_t TASK_task1;
TaskHandle_t TASK_task2;

// current motor Direction
bool motorCurDirR = false; // false->forward
bool motorCurDirL = false; // false->forward
// last motor interrupt time
long motorLastTimeR = 0;
long motorLastTimeL = 0;
// motor encoder counts
long motorCountL = 0;
long motorCountR = 0;

// motor speed values
double motorCurrentRPMR = 0;
double motorCurrentRPML = 0;

String deviceName = "TeenyBot";
// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x0C, 0xB8, 0x15, 0xF8, 0xC2, 0x68};
String success;
esp_now_peer_info_t peerInfo;
ControllerData oldReceived;
ControllerData received;

RobotData robotData;

void leftMotorInterrupt()
{
  motorCurDirL = digitalRead(MOTOR_ENCODER_2B);
  // track the encoder counts with respect to the direction it's turning
  motorCurDirL ? motorCountL-- : motorCountL++;

  // calc the rpm
  uint32_t currentTime = micros();
  if (motorLastTimeL < currentTime)
  {
    // did not wrap around
    double rev = currentTime - motorLastTimeL; // us
    rev = 1.0 / rev;                           // rev per us
    rev *= 1000000;                            // rev per sec
    rev *= 60;                                 // rev per min
    rev /= MOTOR_GEARING;                      // account for gear ratio
    rev /= MOTOR_ENCODER_MULT;                 // account for multiple ticks per rotation
    motorCurrentRPML = rev;
  }
  motorLastTimeL = currentTime;
  // Serial.println("left interupt");
}

void rightMotorInterrupt()
{
  motorCurDirR = digitalRead(MOTOR_ENCODER_1B);
  // track the encoder counts with respect to the direction it's turning
  motorCurDirR ? motorCountR-- : motorCountR++;

  // calc the rpm
  uint32_t currentTime = micros();
  if (motorLastTimeR < currentTime)
  {
    // did not wrap around
    double rev = currentTime - motorLastTimeR; // us
    rev = 1.0 / rev;                           // rev per us
    rev *= 1000000;                            // rev per sec
    rev *= 60;                                 // rev per min
    rev /= MOTOR_GEARING;                      // account for gear ratio
    rev /= MOTOR_ENCODER_MULT;                 // account for multiple ticks per rotation
    motorCurrentRPMR = rev;
  }
  motorLastTimeR = currentTime;
  // Serial.println("right interupt");
}

void initMotorEncoders()
{
  // set the motor direction state
  motorCurDirR = false;
  motorCurDirL = false;
  // set the encoder read input pins
  //  Right:
  pinMode(MOTOR_ENCODER_1A, INPUT_PULLUP);
  pinMode(MOTOR_ENCODER_1B, INPUT_PULLUP);
  // left:
  pinMode(MOTOR_ENCODER_2A, INPUT_PULLUP);
  pinMode(MOTOR_ENCODER_2B, INPUT_PULLUP);

  // attach the interrupts to an encoder pin
  attachInterrupt(MOTOR_ENCODER_1A, rightMotorInterrupt, RISING);
  attachInterrupt(MOTOR_ENCODER_2A, leftMotorInterrupt, RISING);
}

void driveMotor(int motor_pin, int pwm_channel, int pwm_signal){
  // print the pwm_signal
  // SERIAL.print("PWM Signal:");
  // SERIAL.println(pwm_signal);
  digitalWrite(motor_pin, LOW);
  // set the pwm signal for driving the motor
  pwmWrite(pwm_channel, pwm_signal);
}

void sendRobotData(RobotData* data) {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) data, sizeof((*data)));
  if (result == ESP_OK) {
    // Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void task1(void *pvParameters)
{
  SERIAL.print("Task1 running on core: ");
  SERIAL.println(xPortGetCoreID());
  // run task code in infinite loop
  for (;;)
  {
    // Put code to run on the core here
    // check if the received data has changed
    if(!compareControllerData(oldReceived, received)){
      // Check what buttons where pressed and see if we need to do anything
      // button 1 is pressed -> limit the speed to half
      if (received.button1){
        robotData.maxSpeed = (uint8_t)(MAX_PWM/2);
      }else{
        robotData.maxSpeed = MAX_PWM;
      }
      // button 2 is pressed -> 
      if (received.button2){
        // Do something here
      }
      // button 3 is pressed -> 
      if (received.button3){
        // Do something here
      }
      Serial.println("Recieved Data changed");
    }

    sendRobotData(&robotData);
    vTaskDelay(100);
  }
}

void task2(void *pvParameters)
{
  SERIAL.print("Task2 running on core: ");
  SERIAL.println(xPortGetCoreID());
  // run task code in infinite loop
  for (;;)
  {
    // Put code to run on the core here

      // Process the received data
      // int x = received.x;
      // int y = received.y;
      int x  = map(received.x, 0, 4095, -robotData.maxSpeed, robotData.maxSpeed);
      int y  = -map(received.y, 0, 4095, -robotData.maxSpeed, robotData.maxSpeed);

      // Map joystick values to PWM signal values
      robotData.curSpeedL  = constrain(y - x, -robotData.maxSpeed, robotData.maxSpeed);
      robotData.curSpeedR = constrain(y + x, -robotData.maxSpeed, robotData.maxSpeed);

      int lD, rD;
      // get the direction to spin the left motor
      if (robotData.curSpeedL > 0){
        lD = 1;
      }else if (robotData.curSpeedL < 0){
        lD = -1;
      }else{
        lD = 0;
      }
      // get the direction to spin the right motor
      if (robotData.curSpeedR > 0){
        rD = 1;
      }else if (robotData.curSpeedR < 0){
        rD = -1;
      }else{
        rD = 0;
      }

      // Serial.print("lDL, lDR: ");
      // Serial.print(robotData.curSpeedL*lD);
      // Serial.print(",");
      // Serial.print(robotData.curSpeedR*rD);
      // Serial.print("\tL, R: ");
      // Serial.print(robotData.curSpeedL);
      // Serial.print(",");
      // Serial.print(robotData.curSpeedR);
      // Serial.print("\tlD, rD: ");
      // Serial.print(lD);
      // Serial.print(",");
      // Serial.print(rD);
      // Serial.print("\tx, y: ");
      // Serial.print(x);
      // Serial.print(",");
      // Serial.print(y);
      // Serial.println();

      if ( rD > 0){
        driveMotor(MOTOR_1A, MOTOR_1A_PWM_CHANNEL, robotData.curSpeedR*rD);
      }else if (rD < 0){
        driveMotor(MOTOR_1B, MOTOR_1B_PWM_CHANNEL, robotData.curSpeedR*rD);
      }else{
        // Stop the motors
        driveMotor(MOTOR_1A, MOTOR_1A_PWM_CHANNEL, 0);
        driveMotor(MOTOR_1B, MOTOR_1B_PWM_CHANNEL, 0);
      }
      if (lD > 0){
        driveMotor(MOTOR_2A, MOTOR_2A_PWM_CHANNEL, robotData.curSpeedL*lD);
      }else if (lD < 0){
        driveMotor(MOTOR_2B, MOTOR_2B_PWM_CHANNEL, robotData.curSpeedL*lD);
      }else{
        // Stop the motor
        driveMotor(MOTOR_2A, MOTOR_2A_PWM_CHANNEL, 0);
        driveMotor(MOTOR_2B, MOTOR_2B_PWM_CHANNEL, 0);
      }
    vTaskDelay(10);
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t  *incomingData, int len) {
  oldReceived = received;
  memcpy(&received, incomingData, sizeof(received));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  
  // Print the controller data
  Serial.print(received.button1);
  Serial.print(",");
  Serial.print(received.button2);
  Serial.print(",");
  Serial.print(received.button3);
  Serial.print(",");
  Serial.print(received.x);
  Serial.print(",");
  Serial.print(received.y);
  Serial.print(",");
  Serial.print(received.failedCount);
  Serial.print(",");
  Serial.print(received.connected);
  Serial.print(",");
  Serial.print(robotData.failedCount);
  Serial.print(",");
  Serial.print(robotData.connected);
  Serial.print(",");
  Serial.print(robotData.maxSpeed);
  Serial.print(",");
  Serial.print(robotData.curSpeedL);
  Serial.print(",");
  Serial.print(robotData.curSpeedR);
  Serial.println();
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0){
    // Delivery Success :)
    robotData.failedCount = 0;
    robotData.connected = true;
  }
  else{
    // Delivery Fail :(
    if (robotData.failedCount < 3)
    {
      robotData.failedCount++;
    }else if (robotData.failedCount >= 3){
      robotData.connected = false;
    }
  }
}

void initControllerConnnection(){
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

void setup()
{
  // start the serial connection
  SERIAL.begin(115200);

  // Setup ESP-Now Connection to Controller
  initControllerConnnection();

  // initialise the the motor encoders
  initMotorEncoders();

  // set motor PWM output
  ledcSetup(MOTOR_1A_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcSetup(MOTOR_1B_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcSetup(MOTOR_2A_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcSetup(MOTOR_2B_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR_1A, MOTOR_1A_PWM_CHANNEL);
  ledcAttachPin(MOTOR_1B, MOTOR_1B_PWM_CHANNEL);
  ledcAttachPin(MOTOR_2A, MOTOR_2A_PWM_CHANNEL);
  ledcAttachPin(MOTOR_2B, MOTOR_2B_PWM_CHANNEL);

  // =================================================================
  // Setup the multiple tasks:
  // =================================================================
  // core 0 is in charge of Wifi and BT communication so avoid overloading the core.
  xTaskCreatePinnedToCore(
      task1,       /* Task function. */
      "task1",     /* name of task. */
      10000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      1,           /* priority of the task */
      &TASK_task1, /* Task handle to keep track of created task */
      0            /* pin task to core (Core 0)*/
  );

  xTaskCreatePinnedToCore(
      task2,       /* Task function. */
      "task2",     /* name of task. */
      10000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      1,           /* priority of the task */
      &TASK_task2, /* Task handle to keep track of created task */
      1            /* pin task to core (Core 1)*/
  );
}

void loop()
{ /*PUT NOT CODE HERE*/
}