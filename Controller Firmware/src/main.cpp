#include <Arduino.h>
#include <Bounce2.h>
#include "esp_controller/config.h"

#define SERIAL Serial
#define pwmWrite ledcWrite

#define BUZZER_CHANNEL 0
#define BUZZER_RESOLUTION 8

Bounce2::Button btnGreen = Bounce2::Button();
Bounce2::Button btnYellow = Bounce2::Button();
Bounce2::Button btnRed = Bounce2::Button();

int *joyValues;
bool btnValue[3];

void pwmSetup(int pin, int channel, int frequency, int resolution){
  ledcSetup(channel, frequency, resolution);
  ledcAttachPin(pin, channel);
}

int* readJoyStick(int xPin, int yPin){

  static int values[2]; // 0: X, 1: Y

  values[0] = analogRead(xPin);
  values[1] = analogRead(yPin);
  
  // map the values
  values[0] = map(values[0], 0, 4095, -255, 255);
  values[1] = map(values[1], 0, 4095, -255, 255);

  return values;
}

void updateButtons(){
  btnRed.update();
  if(btnRed.pressed()){
    btnValue[0] = btnValue[0] ? false : true; // toggle the value
  }
  btnYellow.update();
  if(btnYellow.pressed()){
    btnValue[1] = btnValue[1] ? false : true; // toggle the value
  }
  btnGreen.update();
  if(btnGreen.pressed()){
    btnValue[2] = btnValue[2] ? false : true; // toggle the value
  }
}

void setup() {
  SERIAL.begin(115200);
  // pwmSetup(BUZZER_PIN, BUZZER_CHANNEL, 2000, BUZZER_RESOLUTION);

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);  
  pinMode(LED_GREEN_PIN, OUTPUT);

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

  digitalWrite(LED_RED_PIN, HIGH);  
  digitalWrite(LED_YELLOW_PIN, HIGH);  
  digitalWrite(LED_GREEN_PIN, HIGH);  
}

void loop() {
  joyValues = readJoyStick(JOY_X_PIN, JOY_Y_PIN);
  updateButtons();

  SERIAL.print(btnValue[0]);
  SERIAL.print(",");
  SERIAL.print(btnValue[1]);
  SERIAL.print(",");
  SERIAL.print(btnValue[2]);
  SERIAL.print(",");
  SERIAL.print(joyValues[0]);
  SERIAL.print(",");
  SERIAL.print(joyValues[1]);
  SERIAL.println();

  delay(10);
}