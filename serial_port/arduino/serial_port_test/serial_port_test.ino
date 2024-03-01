#include "AtSerial.h"

AtSerial serialport;
unsigned char serial_buffer[255];
int message_payload_length = 0;

void setup() {
  serialport.setInitial(0, 57600, 100);
}

void loop() {

  message_payload_length = serialport.readMessage(&serial_buffer[0]);

  if (message_payload_length > 0) {
    serialport.sendMessage(&serial_buffer[0], message_payload_length);
  }

  delay(100);
}
