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
#define GYROLPF_RATIO 1.0f  // used to weight the new measurement
#define ACCLPF_RATIO 0.01f  // used to weight the new measurement
#define MAGLPF_RATIO 0.05f  // used to weight the new measurement
#define ALTLPF_RATIO 0.05f  // used to weight the new measurement
#define BATLPF_RATIO 0.1f  // used to weight the new measurement
#define CF_RATIO 0.95f  // gyro weight in complementary filter

// Some approximate measurements
 #define MOTOR_FORCE 680.0f  // youtube approx value
//#define MOTOR_FORCE 825  // second youtube approx value
#define ROLL_TORQUE_COEFF 0.135f
#define PITCH_TORQUE_COEFF 0.157f
#define IDEAL_VOLTAGE 12.40f
#define DRONE_WEIGHT 950

#endif
