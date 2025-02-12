#include <Arduino.h>
#define DROGUE_PIN 25
#define MAIN_PIN 12
#define REMOTE_SWITCH 27

#define CHUTE_PULSE_DELAY 1000

void pyro_pins_init();
void arm();
void disarm();
void drogue_pulse();
void main_pulse();

void pyro_pins_init() {
  pinMode(REMOTE_SWITCH, OUTPUT);
  pinMode(DROGUE_PIN, OUTPUT);
  pinMode(MAIN_PIN, OUTPUT);
}

void arm() {
  digitalWrite(REMOTE_SWITCH, HIGH);
}

void disarm() {
  digitalWrite(REMOTE_SWITCH, LOW);
}

void drogue_pulse(){
  digitalWrite(DROGUE_PIN, HIGH);
  delay(CHUTE_PULSE_DELAY);
  digitalWrite(DROGUE_PIN, LOW);
  delay(CHUTE_PULSE_DELAY);
}

void main_pulse() {
  digitalWrite(MAIN_PIN, HIGH);
  delay(CHUTE_PULSE_DELAY);
  digitalWrite(MAIN_PIN, LOW);
  delay(CHUTE_PULSE_DELAY);
}

void setup() {
  Serial.begin(115200);

  pyro_pins_init();

  // Serial.println(digitalRead(REMOTE_SWITCH));
  arm();
}

void loop() {
  drogue_pulse();
  main_pulse(); 
}

