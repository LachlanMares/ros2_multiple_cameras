#include "AtSerial.h"
#include "Arduino.h"

AtSerial::AtSerial() {

}

void AtSerial::setInitial(int _port_number, long _baudrate, int timeout) {
    port_number = _port_number;

    switch (port_number) {
        case 1:
            Serial1.begin(_baudrate);
            if(timeout > 0) {
                Serial1.setTimeout(timeout);
            }
            break;

        case 0:
            Serial.begin(_baudrate);
            if(timeout > 0) {
                Serial.setTimeout(timeout);
            }
            break;
    }
}

int AtSerial::readMessage(unsigned char* ext_buffer) {
    unsigned char length_byte = 1;
    unsigned int bytes_read = 0;

    switch (port_number) {
    
      case 0:
        if(Serial.available() > 1) {
          if(Serial.read() == STX) {
            length_byte = Serial.read();
            if(length_byte > 0) {
              unsigned char read_buffer[length_byte];
              bytes_read = Serial.readBytes(read_buffer, length_byte);

              if (bytes_read == length_byte & read_buffer[length_byte-1] == ETX) {
                memcpy(ext_buffer, &read_buffer, length_byte-1);
              } else {
                length_byte = 0;
              }
            }
          }
        }
        break;

      case 1:
        if(Serial1.available() > 1) {
          if(Serial1.read() == STX) {
            length_byte = Serial1.read();
            if(length_byte > 0) {
              unsigned char read_buffer[length_byte];
              bytes_read = Serial1.readBytes(read_buffer, length_byte);

              if (bytes_read == length_byte & read_buffer[length_byte-1] == ETX) {
                memcpy(ext_buffer, &read_buffer, length_byte-1);
              } else {
                length_byte = 0;
              }
            }
          }
        }
        break;
    }

    return length_byte-1; // Payload length, 0 if nothing, -1 if error
}

void AtSerial::sendMessage(unsigned char* write_buffer, int msg_length) {
    unsigned char packet[msg_length+3];

    packet[0] = STX;
    packet[1] = (unsigned char) msg_length+1;
    packet[msg_length+2] = ETX;
    memcpy(&packet[2], write_buffer, msg_length);

    switch (port_number) {
      case 0:
        Serial.write(packet, msg_length+3);
        break;
      
      case 1:
        Serial1.write(packet, msg_length+3);
        break;
    }
}