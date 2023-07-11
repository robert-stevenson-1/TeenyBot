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
  // SERIAL.println("left interupt");
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
  // SERIAL.println("right interupt");
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
    // SERIAL.println("Sent with success");
  }
  else {
    SERIAL.println("Error sending the data");
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
      // handle the pressed button data
      #pragma region handle_button_states
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
      #pragma endregion handle_button_states

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
    // check if the robot is connected before trying to command the robot
    if (robotData.connected){
      // Process the received data
      int x  = map(received.x, 0, 4095, -robotData.maxSpeed, robotData.maxSpeed);
      int y  = -map(received.y, 0, 4095, -robotData.maxSpeed, robotData.maxSpeed);

      // Map joystick values to PWM signal values
      robotData.curSpeedL  = constrain(y - x, -robotData.maxSpeed, robotData.maxSpeed);
      robotData.curSpeedR = constrain(y + x, -robotData.maxSpeed, robotData.maxSpeed);

      int lD, rD;

      #pragma region Calculate_motor_direction
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
      #pragma endregion Calculate_motor_direction

      // SERIAL.print("lDL, lDR: ");
      // SERIAL.print(robotData.curSpeedL*lD);
      // SERIAL.print(",");
      // SERIAL.print(robotData.curSpeedR*rD);
      // SERIAL.print("\tL, R: ");
      // SERIAL.print(robotData.curSpeedL);
      // SERIAL.print(",");
      // SERIAL.print(robotData.curSpeedR);
      // SERIAL.print("\tlD, rD: ");
      // SERIAL.print(lD);
      // SERIAL.print(",");
      // SERIAL.print(rD);
      // SERIAL.print("\tx, y: ");
      // SERIAL.print(x);
      // SERIAL.print(",");
      // SERIAL.print(y);
      // SERIAL.println();

      // set the motor speeds
      #pragma region Motor_Speed_Set

      // set the right motor speed (with direction)
      if ( rD > 0){
        driveMotor(MOTOR_1A, MOTOR_1A_PWM_CHANNEL, robotData.curSpeedR*rD);
      }else if (rD < 0){
        driveMotor(MOTOR_1B, MOTOR_1B_PWM_CHANNEL, robotData.curSpeedR*rD);
      }else{
        // Stop the Left motor
        driveMotor(MOTOR_1A, MOTOR_1A_PWM_CHANNEL, 0);
        driveMotor(MOTOR_1B, MOTOR_1B_PWM_CHANNEL, 0);
      }
      // set the left motor speed (with direction)
      if (lD > 0){
        driveMotor(MOTOR_2A, MOTOR_2A_PWM_CHANNEL, robotData.curSpeedL*lD);
      }else if (lD < 0){
        driveMotor(MOTOR_2B, MOTOR_2B_PWM_CHANNEL, robotData.curSpeedL*lD);
      }else{
        // Stop the Right motor
        driveMotor(MOTOR_2A, MOTOR_2A_PWM_CHANNEL, 0);
        driveMotor(MOTOR_2B, MOTOR_2B_PWM_CHANNEL, 0);
      }
      #pragma endregion Motor_Speed_Set

    }else{
      // stop the robot from driving around when not connected
      driveMotor(MOTOR_1A, MOTOR_1A_PWM_CHANNEL, 0);
      driveMotor(MOTOR_1B, MOTOR_1B_PWM_CHANNEL, 0);
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
  // SERIAL.print("Bytes received: ");
  // SERIAL.println(len);
  
  // Print the controller data
  SERIAL.print(received.button1);
  SERIAL.print(",");
  SERIAL.print(received.button2);
  SERIAL.print(",");
  SERIAL.print(received.button3);
  SERIAL.print(",");
  SERIAL.print(received.x);
  SERIAL.print(",");
  SERIAL.print(received.y);
  SERIAL.print(",");
  SERIAL.print(received.failedCount);
  SERIAL.print(",");
  SERIAL.print(received.connected);
  SERIAL.print(",");
  SERIAL.print(robotData.failedCount);
  SERIAL.print(",");
  SERIAL.print(robotData.connected);
  SERIAL.print(",");
  SERIAL.print(robotData.maxSpeed);
  SERIAL.print(",");
  SERIAL.print(robotData.curSpeedL);
  SERIAL.print(",");
  SERIAL.print(robotData.curSpeedR);
  SERIAL.println();
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // SERIAL.print("\r\nLast Packet Send Status: ");
  // SERIAL.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
  SERIAL.print("MAC Address: ");
  SERIAL.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    SERIAL.println("Error initializing ESP-NOW");
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
    SERIAL.println("Failed to add peer");
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