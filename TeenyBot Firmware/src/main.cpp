#include <Arduino.h>
#include "TeenyBot/Config.h"

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

void task1(void *pvParameters)
{
  SERIAL.print("Task1 running on core: ");
  SERIAL.println(xPortGetCoreID());
  // run task code in infinite loop
  for (;;)
  {
    SERIAL.print(motorCurrentRPML);
    SERIAL.print(", ");
    SERIAL.print(motorCurrentRPMR);
    SERIAL.println();
    
    vTaskDelay(10);
  }
}

void task2(void *pvParameters)
{
  delay(1000);
  SERIAL.print("Task2 running on core: ");
  SERIAL.println(xPortGetCoreID());
  // run task code in infinite loop
  for (;;)
  {
    digitalWrite(MOTOR_1A, HIGH);
    pwmWrite(MOTOR_1A_PWM_CHANNEL, 255);
    pwmWrite(MOTOR_2A_PWM_CHANNEL, 255);
    vTaskDelay(2000);
    digitalWrite(MOTOR_1A, LOW);
    pwmWrite(MOTOR_1A_PWM_CHANNEL, 255);
    pwmWrite(MOTOR_2A_PWM_CHANNEL, 255);
    vTaskDelay(2000);
  }
}

void setup()
{
  // start the serial connection
  SERIAL.begin(115200);

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