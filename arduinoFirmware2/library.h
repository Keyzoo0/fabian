//======================= motor =========================

#define MTR1A 17
#define MTR1B 16

#define MTR2A 26
#define MTR2B 27

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
#define PWM_RESOLUTION 8

//================rosserial ===========================
float rpm1 , rpm2 ;
