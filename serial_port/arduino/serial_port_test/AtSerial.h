#ifndef AtSerial_h
#define AtSerial_h

#include "Arduino.h"

#define STX                                     0x02
#define ETX                                     0x03

class AtSerial {
	
public:
  AtSerial();
  void setInitial(int, long, int);
  int readMessage(unsigned char*);
  void sendMessage(unsigned char*, int);

private:
	int port_number;

};

#endif