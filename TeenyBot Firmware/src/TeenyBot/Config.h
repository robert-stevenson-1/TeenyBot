// Robot's Motor configuration
/* CONNECTIONS:
 *
 * ESP32 D33 - Motor Driver PWM 1A Input
 * ESP32 D32 - Motor Driver PWM 1B Input
 * ESP32 D25 - Motor Driver PWM 2A Input
 * ESP32 D26 - Motor Driver PWM 2B Input
 * ESP32 GND - Motor Driver GND
 */

#define MOTOR_1A 32 // ORANGE wire, right motor
#define MOTOR_1B 33 // YELLOW wire, right motor
#define MOTOR_2A 26 // PURPLE wire, left motor
#define MOTOR_2B 25 // BLUE wire, left motor
#define MOTOR_1A_PWM_CHANNEL 0 // PWM Channel
#define MOTOR_1B_PWM_CHANNEL 1 // PWM Channel
#define MOTOR_2A_PWM_CHANNEL 2 // PWM Channel
#define MOTOR_2B_PWM_CHANNEL 3 // PWM Channel

#define MOTOR_ENCODER_1A 13 //right encoder
#define MOTOR_ENCODER_2B 12 //right encoder
#define MOTOR_ENCODER_2A 27 //left encoder
#define MOTOR_ENCODER_1B 14 //left encoder

#define MOTOR_GEARING     150
#define MOTOR_ENCODER_MULT 7

#define MOTOR_PWM_FREQ 40000
#define MOTOR_PWM_RESOLUTION 8

#define MOTOR_RPM_MAX 60 // Theoretical Max ~105, Measured Theoretical Max ~75
#define MAX_PWM 255