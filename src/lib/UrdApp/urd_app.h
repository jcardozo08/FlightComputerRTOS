#pragma once

#include <Arduino.h>

enum UrdAppMode
{
    URD_APP_SIMULATION,
    URD_APP_STATIC_TEST,
    URD_APP_GS,
    URD_APP_GS_WITH_LORA_CHANGE,
    URD_APP_GS_FLIGHT_COMPUTER_SIMULATION};

class UrdApp
{
private:
    
    int baud;

    float pressure;
    float height;
    float initialHeight;
    float initialPressure;
    float maxHeight;

    int ledPin;
    bool ledEnabled;
    bool ledActivate;

    int buzzerPin;
    bool buzzerEnabled;
    bool buzzerToneMode;
    bool buzzerActivate;
    int buzzerFrequency;

    UrdAppMode operationMode;

    bool appStarted;
    bool readyReceived;
    bool simulationStarted;
    bool gsStarted;

    Stream* loraSerial = nullptr;
    bool startReq = false;
    bool startChange = false;
    bool statusFreq = true;
    void sendLoraMessage(const String& msg);
    void sendLoraMessageln(const String& msg);

    // UTILS METHODS
    bool isNumericMessage(String message);

    void playDot();
    void playDash();

    void buzzerOn();
    void buzzerOff();

    void ledOn();
    void ledOff();
    void blinkLed(int times, int onTime, int offTime);

    void resetBoard();

    // SIMULATION MODE METHODS
    float getInitialHeight(float pressureValue);
    float convertPressureToHeight(float pressureValue);

public:
    UrdApp(int baudRate = 115200);

    void begin();

    void initLed(
        int ledPinConfig,
        bool ledOnConfig = true
    );

    void initBuzzer(
        int buzzerPinConfig,
        bool buzzerOnConfig = true,
        bool buzzerToneConfig = false,
        int buzzerFrequencyConfig = 2000
    );

    void initLora(Stream& loraSerialConfig);

    void startHandshake(UrdAppMode mode);

    void processMessage(String message);

    String getDataString();
    String getHeaderString();

    float getInitialPressure();
    float getInitialHeight();
    bool  updateSimulation();
    float getPressure();
    float getHeight();
    float getMaxHeight();
    void restartInitialPressure(float pressureReference, float heightReference = 0.0f);


private:


};