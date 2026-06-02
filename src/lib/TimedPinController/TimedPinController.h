#ifndef TIMED_PIN_CONTROLLER_H
#define TIMED_PIN_CONTROLLER_H

#include <Arduino.h>

class TimedPinController {
private:
    int pin;
    bool pinOn;
    unsigned long interval;
    unsigned long onDuration;
    bool beepOnTone;

    bool pinSeal = false;
    int maxTimes = 0;
    int timesCount = 0;
    unsigned long previousMillis = 0;

    void seal();
    void unseal();

public:
    TimedPinController(int pino, bool on, unsigned long intervalo, unsigned long duracao, bool toneMode = false);

    void loop(unsigned long time);
    void add();
};

#endif
