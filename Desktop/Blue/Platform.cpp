
#include <Servo.h>
#include "Platform.h"

int leftThrottle = 0;
int rightThrottle = 0;

Servo leftT;
Servo rightT;

Platform::Platform()
{
  
}

void Platform::attachPins(int leftTrack, int rightTrack)
{
  _pinL = leftTrack;
  _pinR = rightTrack;
  leftT.attach(_pinL);
  rightT.attach(_pinR);  
}

void Platform::passRaw(int left, int right)
{
  leftThrottle = left;
  rightThrottle = right;
  leftT.write(leftThrottle);
  rightT.write(rightThrottle);
}

int* Platform::getVals()
{
  int a[2];
  a[0] = leftThrottle;
  a[1] = rightThrottle;
  return a;
}
