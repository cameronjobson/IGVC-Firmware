/*
  Arduino Mega 2560 code for Sabertooth 2x60 Motor Controller
  Uses Packetized Serial Mode.

  HARDWARE CONNECTIONS (RECOMMENDED ON MEGA 2560):
  Sabertooth Packet Serial:
  - Mega TX2 (Pin 16)  -> Sabertooth S1 (Sabertooth RX)
  - Mega GND           -> Sabertooth 0V (GND)
  (S2 is not required unless you need to read from Sabertooth.)

  FlySky iBUS (FS-IA6B, etc.):
  - Receiver iBUS      -> Mega RX1 (Pin 19)
  - Receiver VCC       -> 5V
  - Receiver GND       -> GND

  DEBUG:
  - USB Serial (Serial) -> PC Serial Monitor

  NOTE:
  - Avoid SoftwareSerial on Mega for high baud (115200).
  - Use hardware UARTs (Serial1/2/3) instead.
*/

#include <IBusBM.h>

// Sabertooth settings
const uint8_t sabertoothAddress = 128; // default address

// iBUS object
IBusBM ibus;

void setup() {
  // Debug to PC
  Serial.begin(115200);
  //Serial.println("Sabertooth 2x60 + iBUS Starting...");

  // iBUS on Serial1 (RX1 pin 19)
  //Serial1.begin(115200);
  ibus.begin(Serial);

  // Sabertooth on Serial2 (TX2 pin 16). RX2 pin 17 not used.
  //Serial2.begin(115200);
  //delay(500);
  //Serial2.write(0xAA); // Sabertooth autobaud sync at 115200

  Serial.println("Ready.");
}

void loop() {
  // --- Demo motor ramp code DISABLED per request ---
  /*
  Serial.println("Ramping motors forward...");
  for (int speed = 0; speed <= 127; speed++) {
    motor1_forward(speed);
    motor2_forward(speed);
    delay(20);
  }
  delay(1000);

  Serial.println("Ramping motors to stop...");
  for (int speed = 127; speed >= 0; speed--) {
    motor1_forward(speed);
    motor2_forward(speed);
    delay(20);
  }
  delay(1000);

  Serial.println("Ramping motors reverse...");
  for (int speed = 0; speed <= 127; speed++) {
    motor1_reverse(speed);
    motor2_reverse(speed);
    delay(20);
  }
  delay(1000);

  Serial.println("Stopping motors...");
  stop_motors();
  delay(2000);
  */

  // Placeholder: read a few iBUS channels so you can confirm it's working
  int ch1 = ibus.readChannel(0);
  int ch2 = ibus.readChannel(1);
  int ch3 = ibus.readChannel(2);
  int ch4 = ibus.readChannel(3);

  Serial.print("CH1="); Serial.print(ch1);
  Serial.print(" CH2="); Serial.print(ch2);
  Serial.print(" CH3="); Serial.print(ch3);
  Serial.print(" CH4="); Serial.println(ch4);

  delay(50);
}

// --- Motor Control Functions ---

// Drive Motor 1 Forward: speed 0..127
void motor1_forward(uint8_t speed) { sendCommand(0, speed); }

// Drive Motor 1 Reverse: speed 0..127
void motor1_reverse(uint8_t speed) { sendCommand(1, speed); }

// Drive Motor 2 Forward: speed 0..127
void motor2_forward(uint8_t speed) { sendCommand(4, speed); }

// Drive Motor 2 Reverse: speed 0..127
void motor2_reverse(uint8_t speed) { sendCommand(5, speed); }

// Stop both motors
void stop_motors() {
  motor1_forward(0);
  motor2_forward(0);
}

// Helper function to send packetized serial command to Sabertooth
void sendCommand(uint8_t command, uint8_t value) {
  // Packetized Serial format:
  // 1) Address
  // 2) Command
  // 3) Value
  // 4) Checksum = (Address + Command + Value) & 0x7F
  uint8_t checksum = (sabertoothAddress + command + value) & 0x7F;

  Serial2.write(sabertoothAddress);
  Serial2.write(command);
  Serial2.write(value);
  Serial2.write(checksum);
}