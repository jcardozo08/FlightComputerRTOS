#include "TimedPinController.h"

TimedPinController::TimedPinController(int pino, bool on, unsigned long intervalo, unsigned long duracao, bool toneMode)
    : pin(pino), pinOn(on), interval(intervalo), onDuration(duracao), beepOnTone(toneMode) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, !pinOn);
}

void TimedPinController::loop(unsigned long time) {
    seal();
    if (pinSeal) {
        if (timesCount < maxTimes) {
            if (time <= previousMillis + onDuration) {
                if (!beepOnTone) {
                    digitalWrite(pin, pinOn);
                } else {
                    tone(pin, 1000);
                }
            } else if (time > previousMillis + onDuration && time <= previousMillis + onDuration + interval) {
                if (!beepOnTone) {
                    digitalWrite(pin, !pinOn);
                } else {
                    noTone(pin);
                }
            } else {
                previousMillis = time;
                timesCount += 1;
            }
        } else {
            timesCount = 0;
            maxTimes = 0;
            unseal();
        }
    }
}

void TimedPinController::add() {
    if (!pinSeal) {
        maxTimes += 1;
    }
}

void TimedPinController::seal() {
    if (!pinSeal && maxTimes != 0) {
        previousMillis = millis();
        pinSeal = true;
    }
}

void TimedPinController::unseal() {
    pinSeal = false;
}
