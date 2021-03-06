#ifndef LIDARLite_h
#define LIDARLite_h

#define LIDARLITE_ADDR_DEFAULT 0x62

#include <Arduino.h>

class LIDARLite
{
  public:
      LIDARLite();
      void begin(int = 0, bool = false, char = LIDARLITE_ADDR_DEFAULT);
      void configure(int = 0, char = LIDARLITE_ADDR_DEFAULT);
      void setI2Caddr(char, char, char = LIDARLITE_ADDR_DEFAULT);
      void reset(char = LIDARLITE_ADDR_DEFAULT);
      int distance(bool = true, char = LIDARLITE_ADDR_DEFAULT);
      void write(char, char, char = LIDARLITE_ADDR_DEFAULT);
      void read(char, int, byte*, bool, char);
      void correlationRecordToSerial(char = '\n', int = 256, char = LIDARLITE_ADDR_DEFAULT);
};

#endif
