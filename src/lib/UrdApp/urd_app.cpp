#include "URD_APP.h"

#if defined(ARDUINO_ARCH_AVR)
#include <avr/wdt.h>
#endif

UrdApp::UrdApp(int baudRate)
{
    baud = baudRate;

    pressure = 0.0f;
    height = 0.0f;
    initialHeight = 0.0f;
    initialPressure = 0.0f;

    ledEnabled = false;
    ledActivate = true;

    buzzerEnabled = false;
    buzzerToneMode = false;
    buzzerActivate = true;
    buzzerFrequency = 2000;

    appStarted = false;
    readyReceived = false;
    simulationStarted = false;
    gsStarted = false;
}

void UrdApp::initLed(
    int ledPinConfig,
    bool ledOnConfig
)
{
    ledEnabled = true;
    ledPin = ledPinConfig;
    ledActivate = ledOnConfig;

    if (ledEnabled && ledPin >= 0)
    {
        pinMode(ledPin, OUTPUT);
        ledOff();
    }
}

void UrdApp::ledOn()
{
    if (!ledEnabled || ledPin < 0)
    {
        return;
    }

    digitalWrite(ledPin, ledActivate ? HIGH : LOW);
}

void UrdApp::ledOff()
{
    if (!ledEnabled || ledPin < 0)
    {
        return;
    }

    digitalWrite(ledPin, ledActivate ? LOW : HIGH);
}

void UrdApp::blinkLed(int times, int onTime, int offTime)
{
    if (!ledEnabled || ledPin < 0)
    {
        return;
    }

    for (int i = 0; i < times; i++)
    {
        ledOn();
        delay(onTime);

        ledOff();
        delay(offTime);
    }
}

void UrdApp::initBuzzer(
    int buzzerPinConfig,
    bool buzzerOnConfig,
    bool buzzerToneConfig,
    int buzzerFrequencyConfig
)
{
    buzzerEnabled = true;
    buzzerPin = buzzerPinConfig;
    buzzerToneMode = buzzerToneConfig;
    buzzerActivate = buzzerOnConfig;
    buzzerFrequency = buzzerFrequencyConfig;

    if (buzzerEnabled && buzzerPin >= 0)
    {
        pinMode(buzzerPin, OUTPUT);
        buzzerOff();
    }
}

void UrdApp::playDot()
{
    if (!buzzerEnabled || buzzerPin < 0)
    {
        return;
    }

    buzzerOn();
    delay(80);

    buzzerOff();
    delay(80);
}

void UrdApp::playDash()
{
    if (!buzzerEnabled || buzzerPin < 0)
    {
        return;
    }

    buzzerOn();
    delay(250);

    buzzerOff();
    delay(120);
}

void UrdApp::buzzerOn()
{
    if (!buzzerEnabled || buzzerPin < 0)
    {
        return;
    }

    if (buzzerToneMode)
    {
        tone(buzzerPin, buzzerFrequency);
    }
    else
    {
        digitalWrite(buzzerPin, buzzerActivate ? HIGH : LOW);
    }
}

void UrdApp::buzzerOff()
{
    if (!buzzerEnabled || buzzerPin < 0)
    {
        return;
    }

    if (buzzerToneMode)
    {
        noTone(buzzerPin);
    }
    else
    {
        digitalWrite(buzzerPin, buzzerActivate ? LOW : HIGH);
    }
}

void UrdApp::initLora(Stream& loraSerialConfig)
{
    loraSerial = &loraSerialConfig;
}

void UrdApp::sendLoraMessageln(const String& msg)
{
    if (loraSerial == nullptr)
    {
        return;
    }

    loraSerial->println(msg);
}

void UrdApp::sendLoraMessage(const String& msg)
{
    if (loraSerial == nullptr)
    {
        return;
    }

    loraSerial->print(msg);
}

void UrdApp::begin()
{
    Serial.begin(baud);

    delay(500);

    if (ledEnabled)
    {
        blinkLed(2, 100, 100);
    }
}

void UrdApp::startHandshake(UrdAppMode mode)
{
    operationMode = mode;

    appStarted = false;
    readyReceived = false;
    simulationStarted = false;
    gsStarted = false;

    unsigned long handshakeTime = millis();

    if (buzzerEnabled)
    {
        playDot();
        playDot();
        playDot();
        delay(1000);
    }

    Serial.println(F("Starting URD App Serial Communication..."));

    while (!appStarted)
    {
        if (buzzerEnabled && !readyReceived && (millis() - handshakeTime) >= 5000)
        {
            playDot();
            playDot();
            handshakeTime = millis();
        }

        if (Serial.available())
        {
            String message = Serial.readStringUntil('\n');
            message.trim();

            processMessage(message);
        }

        delay(10);
    }

    ledOn();
}

void UrdApp::processMessage(String message)
{
    if (message.length() == 0)
    {
        return;
    }

    if (message == "READY")
    {
        Serial.println(F("OK"));
        readyReceived = true;

        if (buzzerEnabled)
        {
            playDot();
            playDot();
        }

        return;
    }

    if (operationMode == URD_APP_GS || operationMode == URD_APP_GS_FLIGHT_COMPUTER_SIMULATION || operationMode == URD_APP_GS_WITH_LORA_CHANGE) {
        if (message == "GPS_COORDS")
        {
            Serial.println(F("~\t~"));

            appStarted = true;
            simulationStarted = false;
            gsStarted = true;

            if (operationMode == URD_APP_GS_FLIGHT_COMPUTER_SIMULATION) {
                Serial.println(F("---Flight Computer RTOS---"));
            } else if (operationMode == URD_APP_GS) {
                Serial.println(F("---Ground Station UFABC Rocket Design---"));
            } else if (operationMode == URD_APP_GS_WITH_LORA_CHANGE) {
                Serial.println(F("---Ground Station UFABC Rocket Design with Lora Change---"));
            }

            if (buzzerEnabled)
            {
                playDash();
            }

            return;
        }
    }

    if (operationMode == URD_APP_SIMULATION)
    {
        if (message == "SIMULATION")
        {
            Serial.println(F("STARTED"));

            appStarted = true;
            simulationStarted = true;
            gsStarted = false;

            getInitialPressure();

            if (buzzerEnabled)
            {
                playDash();
            }

            return;
        }

    }

    if (operationMode == URD_APP_GS_WITH_LORA_CHANGE)
    {
        if (message == "MUD4R_FR3Q")
        {
            sendLoraMessageln("MUD4R_FR3Q");
            startReq = true;
            return;
        }
        if (message == "MUD4R_OK" && startReq)
        {   
            // changeLoraFrequency();
            startChange = true;
            return;
        }
    }

    if (message == "RST")
    {
        Serial.println(F("RESETTING"));
        delay(100);

        resetBoard();

        return;
    }


}

// UTILS METHODS
bool UrdApp::isNumericMessage(String message)
{
    bool hasDigit = false;
    bool hasDot = false;

    for (int i = 0; i < message.length(); i++)
    {
        char character = message[i];

        if (character >= '0' && character <= '9')
        {
            hasDigit = true;
        }

        else if (character == '.' && !hasDot)
        {
            hasDot = true;
        }

        else if (character == '-' && i == 0)
        {
            continue;
        }

        else
        {
            return false;
        }
    }

    return hasDigit;
}



void UrdApp::resetBoard()
{
#if defined(ARDUINO_ARCH_ESP32)
    ESP.restart();
#elif defined(ARDUINO_ARCH_AVR)
    wdt_enable(WDTO_15MS);
    while (true) {}
#else
    Serial.println(F("Board reset not supported on this platform."));
#endif
}

// --- SIMULATION MODE METHODS ---
bool UrdApp::updateSimulation()
{
    if (Serial.available())
    {
        String message = Serial.readStringUntil('\n');
        message.trim();

        if (isNumericMessage(message)) {
            pressure = message.toFloat();
            height = convertPressureToHeight(pressure);
            if (height > maxHeight) {
                maxHeight = height;
            }
            return true;
        }
    }

    return false;
}

float UrdApp::getInitialPressure()
{
    float sum = 0.0f;
    int counter = 0;

    for (int i = 0; i < 10; i++)
    {
        while (!Serial.available())
        {
            
            delay(1);
        }

        float pressureValue = Serial.parseFloat();

        sum += pressureValue;
        counter++;
    }

    initialPressure = sum / counter;
    initialHeight = getInitialHeight(initialPressure);
    pressure = initialPressure;
    height = 0.0f;

    Serial.print(F("Simulator - Initial Pressure / Initial Height: "));
    Serial.print(initialPressure);
    Serial.print(F(" Pa \t"));
    Serial.print(initialHeight);
    Serial.println(F(" m"));

    return initialPressure;
}

float UrdApp::getInitialHeight()
{
    return initialHeight;
}

float UrdApp::getMaxHeight()
{
    return maxHeight;
}

float UrdApp::getPressure()
{
    return pressure;
}

float UrdApp::getHeight()
{
    return height;
}

float UrdApp::convertPressureToHeight(float pressureValue)
{
    height = 44330.0f * 
             (1.0f - powf(pressureValue / 101325.0f, 1.0f / 5.25588f)) 
             - initialHeight;

    return height;
}

float UrdApp::getInitialHeight(float pressureValue)
{
    float absoluteHeight = 44330.0f *
                           (1.0f - powf(pressureValue / 101325.0f, 1.0f / 5.25588f));

    return absoluteHeight;
}

void UrdApp::restartInitialPressure(float pressureReference, float heightReference)
{
    initialPressure = pressureReference;
    initialHeight = getInitialHeight(initialPressure) - heightReference;

    pressure = pressureReference;
    height = 0.0f;

    Serial.print(F("Simulator restarted - Initial Pressure / Initial Height: "));
    Serial.print(initialPressure);
    Serial.print(F(" Pa \t"));
    Serial.print(initialHeight);
    Serial.println(F(" m"));
}

String UrdApp::getDataString()
{
    String data;

    data += String(pressure, 2);
    data += '\t';

    data += String(height, 2);
    data += '\t';

    return data;
}

String UrdApp::getHeaderString()
{
    String header;

    header += F("sim_press_pa\t");
    header += F("sim_height_m\t");

    return header;
}