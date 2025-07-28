//======================= motor =========================

#define MTR1A 17
#define MTR1B 16

#define MTR2A 27
#define MTR2B 26

// Deklarasi variabel global untuk menyimpan nilai PWM
volatile int pwm1 = 0;  // PWM untuk motor 1
volatile int pwm2 = 0;  // PWM untuk motor 2

// Definisikan kanal PWM untuk motor 1 dan motor 2
#define MTR1A_PWM_CHANNEL 0
#define MTR1B_PWM_CHANNEL 1
#define MTR2A_PWM_CHANNEL 2
#define MTR2B_PWM_CHANNEL 3

// Definisikan frekuensi PWM dan resolusi
#define PWM_FREQ 20000
#define PWM_RESOLUTION 10

// ========================= encoder =============

#include <Arduino.h>

// Encoder Pins
#define ENC1A 13
#define ENC1B 25
#define ENC2A 18
#define ENC2B 19

// Define LED pin for debugging purposes
#define LED 2

// Variables to store encoder count
volatile long countEnc_A = 0;
volatile long countEnc_B = 0;
volatile long pulseL = 0; // Motor 1 pulse count
volatile long pulseR = 0; // Motor 2 pulse count

// Variables for RPM calculation
volatile long rpmMotor1 = 0;  // RPM for motor 1
volatile long rpmMotor2 = 0;  // RPM for motor 2

// Timer interval (1 second)
unsigned long lastMillisMotor1 = 0;
unsigned long lastMillisMotor2 = 0;
const long interval = 1000;  // 1 second
