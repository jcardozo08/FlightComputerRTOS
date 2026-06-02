#include "buzzer.h"

Buzzer::Buzzer(int pin, int lPin, unsigned long intervalMillis, bool on, bool toneMode) : buzzerPin(pin), ledPin(lPin), interval(intervalMillis), buzzOn(on), toneMode(toneMode)
{
}

void Buzzer::begin()
{
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  if (toneMode)
  {
    noTone(buzzerPin);
  }
  else
  {
    digitalWrite(buzzerPin, !buzzOn);
  }
}

void Buzzer::beep(unsigned long currentTime)
{
  if (estadoBeep)
  {
    if (estadoBeepInterno)
    {
      if (toneMode)
      {
        tone(buzzerPin, 1000);
      }
      else
      {
        digitalWrite(buzzerPin, buzzOn);
      }
      digitalWrite(ledPin, HIGH);
    }
    else
    {
      if (toneMode)
      {
        noTone(buzzerPin);
      }
      else
      {
        digitalWrite(buzzerPin, !buzzOn);
      }
      digitalWrite(ledPin, LOW);
    }

    if (beepCount % 2 == 0)
    {
      beepDuration = 250;
    }
    else
    {
      if (beepPattern[readIndex] == false)
      {
        beepDuration = 600;
      }
      else
      {
        beepDuration = 200;
      }
    }

    if (currentTime - previousMillisInterno > beepDuration)
    {
      estadoBeepInterno = !estadoBeepInterno;
      previousMillisInterno = millis();
      if (beepCount % 2 != 0)
      {
        readIndex++;
      }
      if (++beepCount > maxBeeps * 2)
      {
        beepCount = 0;
        estadoBeep = !estadoBeep;
        estadoBeepInterno = false;
      }
    }
  }
  else
  {
    if (toneMode)
    {
      noTone(buzzerPin);
    }
    else
    {
      digitalWrite(buzzerPin, !buzzOn);
    }
    digitalWrite(ledPin, LOW);
  }
  if (currentTime - previousMillis > interval)
  {
#if DEBUG_PRINTS
    for (int i = 0; i < SYSTEM_SENSORS_ON; ++i)
    {
      Serial.print(beepPattern[i] ? '.' : '-');
    }
    Serial.println(); 
#endif // DEBUG
    estadoBeep = !estadoBeep;
    previousMillis = millis();
    previousMillisInterno = previousMillis;
    readIndex = 0;
  }
}

void Buzzer::addSystem(bool booleano)
{
  beepPattern[writeIndex] = booleano;

  writeIndex = (writeIndex + 1) % maxBeeps;
}
