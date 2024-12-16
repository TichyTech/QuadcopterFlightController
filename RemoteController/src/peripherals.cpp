#include "peripherals.h"

void setup_pins(){
  pinMode(POT_PIN, INPUT);
  pinMode(JOYLB_PIN, INPUT);
  pinMode(JOYLY_PIN, INPUT);
  pinMode(JOYLX_PIN, INPUT);
  pinMode(JOYRY_PIN, INPUT);
  pinMode(JOYRX_PIN, INPUT);
  pinMode(BUTTON_L_PIN, INPUT);
  pinMode(BUTTON_R_PIN, INPUT);
  pinMode(SWITCH_PIN, INPUT);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(WSLED_PIN, OUTPUT);
}