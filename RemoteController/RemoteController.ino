#include <SPI.h>
#include "printf.h"
#include <RF24.h>
#include <nRF24L01.h>

#include "comm_structs.h"

#define HUMAN_READABLE 0
#define PRINT_COMMANDS 0
#define DEBUG 0

#define PRINT_MIN_DELAY 500

RF24 radio(9,10); // CE, CSN

#define POT A0

#define JOYLB_PIN A1
#define JOYLX_PIN A2
#define JOYLY_PIN A3

#define JOYRX_PIN A6
#define JOYRY_PIN A7

#define BUTTON_R 2
#define BUTTON_L 3
#define SWITCH 4

#define LED1 5
#define LED2 6
#define LED3 7
#define WSLED 8

#include <FastLED.h>
CRGB leds[1];

const byte CMDADD[4] = "DRR";  // receive address
const byte STATADD[4] = "DRT";  // transmit address

int16_t Xstart;
int16_t Ystart;
int16_t X2start;
int16_t Y2start;

void setup() {
  pinMode(POT, INPUT);
  pinMode(JOYLB_PIN, INPUT);
  pinMode(JOYLY_PIN, INPUT);
  pinMode(JOYLX_PIN, INPUT);
  pinMode(JOYRY_PIN, INPUT);
  pinMode(JOYRX_PIN, INPUT);
  pinMode(BUTTON_L, INPUT);
  pinMode(BUTTON_R, INPUT);
  pinMode(SWITCH, INPUT);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(WSLED, OUTPUT);
  FastLED.addLeds<WS2812B, WSLED, GRB>(leds, 1);  // GRB ordering is typical

  Xstart = analogRead(JOYLX_PIN);
  Ystart = analogRead(JOYLY_PIN);
  X2start = analogRead(JOYRX_PIN);
  Y2start = analogRead(JOYRY_PIN);
  
  Serial.begin(500000);
  while (!Serial){}

  SPI.setClockDivider(SPI_CLOCK_DIV2);

  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {} 
  }
  
  radio.setPALevel(RF24_PA_MAX, 1);
  radio.setChannel(3);
  radio.setAddressWidth(3);
  radio.setDataRate(RF24_2MBPS);
  radio.enableDynamicAck();
  radio.setAutoAck(0);
  radio.openWritingPipe(CMDADD); 
  radio.openReadingPipe(1, STATADD); 
  radio.startListening();   
  
  printf_begin();
  radio.printPrettyDetails();

  Serial.println("Setup done");
}

float Xval;
float Yval;
float X2val;
float Y2val;
float Tval;
uint8_t button1_state = 1;
uint8_t button2_state = 1;

uint8_t motors_mode = 1;
uint8_t motors_on = 0;
bool led_on = 0;

long last_cmd_t = 0;
uint32_t last_shown = 0;

void loop() {  
  Xval = (analogRead(JOYLX_PIN) - Xstart)/20.0;  // pm 12.5 degs
  Yval = (analogRead(JOYLY_PIN) - Ystart)/20.0;
  X2val = -(analogRead(JOYRX_PIN) - X2start)/20.0;  // pm 12.5 degs
  Y2val = -(analogRead(JOYRY_PIN) - Y2start)/20.0;
  Tval = (analogRead(POT))/1024.0;

  if (digitalRead(BUTTON_L) && !button1_state){ 
    button1_state = 1;
  }
  if (!digitalRead(BUTTON_L) && button1_state){
    button1_state = 0;
    motors_mode = 1 + ((motors_mode) % 5);
    if (DEBUG) Serial.println("Left button pressed");
  }
  
  if (digitalRead(BUTTON_R) && !button2_state){ 
    button2_state = 1;
  }
  if (!digitalRead(BUTTON_R) && button2_state){
    button2_state = 0;
    motors_on = !motors_on;
    if (DEBUG) Serial.println("Right button pressed");
  }

  analogWrite(LED1, int(256*Xval/30));
  analogWrite(LED2, int(256*Yval/30));

  leds[0] = CHSV(0, 255, int(64*!(motors_on)));
  FastLED.show();

  if ( abs(Xval) < 0.5) Xval = 0; // less than 1 deg
  if ( abs(Yval) < 0.5) Yval = 0; // less than 1 deg
  if ( abs(X2val) < 0.5) X2val = 0; // less than 1 deg
  if ( abs(Y2val) < 0.5) Y2val = 0; // less than 1 deg
  
  ctrl_msg_t ctrl_msg = {X2val, Y2val, 0.0, Tval, motors_on*motors_mode};
  msg_t msg; 
  msg.type = 0;
  msg.data.ctrl_data = ctrl_msg;
  bool report = 0;


//  Serial.println(millis() - last_cmd_t);
  if ((millis() - last_cmd_t) >= 100){
    last_cmd_t = millis();
    radio.stopListening(); 
    radio.flush_tx();
    long start_time = micros();
//    report = radio.write(&msg, sizeof(msg_t), 0);       // 800 usec
//    bool report = radio.writeFast(&payload, sizeof(payload));       // 700 usec
    report = radio.startWrite(&msg, sizeof(msg_t), 0); 
    long end_time = micros();
    delayMicroseconds(250);
    radio.startListening();
    radio.flush_rx();
    if (DEBUG) Serial.println("Sending commands"); 
    if (PRINT_COMMANDS){
      Serial.print(Xval);
      Serial.print(" ");
      Serial.print(Yval);
      Serial.print(" ");
      Serial.print(X2val);
      Serial.print(" ");
      Serial.print(Y2val);
      Serial.print(" ");
      Serial.print(Tval);
      Serial.print(" ");
      Serial.print(motors_mode);
      Serial.println(" ");
      Serial.println(motors_on);
    }
//    
//    Serial.println("Transmit took " + String(end_time - start_time) + " microseconds");
  }

  if (Serial.available() > 0 && ((millis() - last_cmd_t) >= 90)){  // received serial commands
    String serial_line = Serial.readStringUntil('\n');
    uint8_t space_loc = serial_line.indexOf(' ');
    String msg_type = serial_line.substring(0, space_loc);
    if (msg_type == "roll" || msg_type == "pitch" || msg_type == "yaw"){  // react only if pitch or roll or yaw is seen
      float serial_reading[5] = {0,0,0,0,0};
      for (int i = 0; i < 5; i++){
        int next_space = serial_line.indexOf(' ', space_loc + 1);
        if (next_space == -1) next_space = serial_line.length() - 1;
        String numero = serial_line.substring(space_loc, next_space);
        serial_reading[i] = numero.toFloat();
        space_loc = next_space;
      } 
      float P = serial_reading[0];
      float I = serial_reading[1];
      float D = serial_reading[2];
      float sat = serial_reading[3];
      float LPc = serial_reading[4];
      config_msg_t new_config;
      if (msg_type == "roll"){
        new_config = {P, I, D, sat, LPc, 0};
        String text = "Sending roll config: P " + String(P, 2) + ", I " + String(I, 2) + ", D " + String(D, 2) + ", sat " + String(sat, 2) + ", LPc " + String(LPc, 2);
        if (DEBUG) Serial.println(text);
      }
      else if (msg_type == "pitch"){
        new_config = {P, I, D, sat, LPc, 1};
        String text = "Sending pitch config: P " + String(P, 2) + ", I " + String(I, 2) + ", D " + String(D, 2) + ", sat " + String(sat, 2) + ", LPc " + String(LPc, 2);
        if (DEBUG) Serial.println(text);
      }
      else if (msg_type == "yaw"){
        new_config = {P, I, D, sat, LPc, 2};
        String text = "Sending yaw config: P " + String(P, 2) + ", I " + String(I, 2) + ", D " + String(D, 2) + ", sat " + String(sat, 2) + ", LPc " + String(LPc, 2);
        if (DEBUG) Serial.println(text);
      }
      // sent config message
      msg_t msg; 
      msg.type = 1;
      msg.data.config_data = new_config;
      bool report = 0;
      radio.stopListening(); 
      report = radio.write(&msg, sizeof(msg_t));
      delayMicroseconds(250);    
      radio.startListening(); 
      if (report && DEBUG){
        Serial.println("Config sent");
      }
    }
    
  }

  uint8_t pipe;
  telemetry_msg_t tele_msg;
  if(radio.available(&pipe)){
    radio.read(&tele_msg, sizeof(tele_msg)); 
    if (tele_msg.type == 0){  // state data

      #if MIN_DELAY > 0  // wait at least MIN_DELAY before printing telemetry
      uint32_t current_t = millis();
      if (current_t - last_shown < PRINT_MIN_DELAY) return;
      last_shown = current_t;
      #endif
      
      if (HUMAN_READABLE){
        char buffr[100];
        state_struct s_data = tele_msg.data.state_data; 
        format_state(s_data, buffr, sizeof(buffr));  //1168 micros
        Serial.println(buffr);  // 828 micros
      }
      else Serial.write((uint8_t*)&tele_msg, sizeof(tele_msg));
    }
    if (tele_msg.type == 1){  // sensor data
      if (HUMAN_READABLE){
        sensor_struct s_data = tele_msg.data.sensor_data;
        Serial.println("Telemetry: " + String(s_data.battery) + " " + String(s_data.height));
      }
      else Serial.write((uint8_t*)&tele_msg, sizeof(tele_msg));
    }
  }
}
