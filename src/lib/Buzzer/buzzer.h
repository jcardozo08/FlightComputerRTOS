#ifndef BUZZER_H
#define BUZZER_H
#include "../config.h"
#include <Arduino.h>

class Buzzer {
  private:
    bool estadoBeep = false;
    bool estadoBeepInterno = false;
    int buzzerPin;
    int ledPin;
    unsigned long previousMillis = 0;
    unsigned long previousMillisInterno = 0;
    unsigned long interval;
    unsigned long beepDuration;
    int beepCount = 0;
    int maxBeeps = SYSTEM_SENSORS_ON;
    bool buzzOn;
    bool toneMode;
    bool beepPattern[SYSTEM_SENSORS_ON];
    int writeIndex = 0;
    int readIndex = 0;

    
  public:
    Buzzer(int pin, int lPin, unsigned long intervalMillis, bool on, bool toneMode);
    
    void begin();

    void beep(unsigned long currentTime);

    void addSystem(bool booleano);
};

#endif
