#ifndef CONFIG
#define CONFIG

// Pin definitions
#define MOTOR1_PIN 15
#define MOTOR2_PIN 14
#define MOTOR3_PIN 2
#define MOTOR4_PIN 3

#define REDLED_PIN 10
#define YELLOWLED_PIN 11
#define GREENLED_PIN 12
#define BLUELED_PIN 13

#define ADC1_PIN 28
#define ADC2_PIN 27

#define SWITCH_PIN 26

// Settings
#define SAFETY 1  // set safe mode
#define TELEMETRY 1 
#define DEBUG 0 // enable debugging messages

// Timeout settings
#define COMMAND_TIMEOUT_MS 60000  // 60000 ms
#define SENSOR_TIMEOUT_US 2000  // 2 ms

// filter constants
#define GYROLPF_RATIO 0.1
#define ACCLPF_RATIO 0.01
#define MAGLPF_RATIO 0.05
#define ALTLPF_RATIO 0.05
#define BATLPF_RATIO 0.1
#define MOTOR_LPF 0.7
#define CF_RATIO 0.9995

// Some approximate measurements
 #define MOTOR_FORCE 680.0  // youtube approx value
//#define MOTOR_FORCE 825  // second youtube approx value
#define ROLL_TORQUE_COEFF 0.135
#define PITCH_TORQUE_COEFF 0.157
#define IDEAL_VOLTAGE 12.40
#define DRONE_WEIGHT 950

#endif
