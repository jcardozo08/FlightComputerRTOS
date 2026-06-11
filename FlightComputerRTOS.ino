//=================================================================================== INITIALIZATIONS ===================================================================================

#include <Arduino.h>
#include <math.h>
#include <Wire.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "src/lib/config.h"
#include "src/lib/pinout.h"

//=================================================================================== REDEFINES CONFIGURATIONS ===================================================================================

  #if (BUZZER_SILENT_MODE)
  constexpr int REAL_BUZZ_PIN = BUZZ_PIN;
  constexpr int REAL_BUZZ_ON = BUZZ_ON;

  #undef BUZZ_PIN
  #undef BUZZ_TONE
  #undef BUZZ_ON

  #define BUZZ_PIN LED_PIN
  #define BUZZ_TONE 0
  #define BUZZ_ON HIGH
  #endif

//=================================================================================== OBJECTS ===================================================================================

//------------------------------ BAROMETER OBJECTS ------------------------------//
#if (BARO)

  #if (USE_BMP180)
    #include "src/lib/Barometro/BMP085.h"
    BMP085 baro;
  #endif

  #if (USE_BMP280)
    #include "src/lib/BMP280/BMP280.h"
    #define baro Barometer
    BMP280 baro;
  #endif

  #if (USE_BME280)
    #include <Bme280.h>
    Bme280TwoWire baro;
  #endif

  #if (USE_BMP380)
    // Placeholder for future implementation
    // #include "src/lib/BMP380/BMP380.h"
    // BMP380 baro;
  #endif

#endif

//------------------------------ ACCELEROMETER OBJECTS ------------------------------//
#if (ACCEL)

  #if (USE_MPU9250_ACCEL)
    #include "src/lib/MPU9250_ACCEL/MPU9250_ACCEL.h"
    #define accel Accelerometer
    MPU9250_ACCEL accel(16);
  #endif

  #if (USE_ICM20948_ACCEL) || (USE_ICM20948_GYRO) || (USE_ICM20948_MAG)
    #include <ICM20948_WE.h>
    #ifndef ICM20948_ADDR
      #define ICM20948_ADDR 0x68
    #endif
    ICM20948_WE imu = ICM20948_WE(ICM20948_ADDR);
  #endif

#endif

//------------------------------ GYROSCOPE OBJECTS ------------------------------//
#if (GYRO)

  #if (USE_MPU9250_GYRO)
    #include "src/lib/MPU9250_GYRO/MPU9250_GYRO.h"
    #define gyro Gyroscope
    MPU9250_GYRO gyro(2000);
  #endif

#endif

//------------------------------ MAGNETOMETER OBJECTS ------------------------------//
#if (MAG)

  #if (USE_AK8963_MAG)
    #include "src/lib/AK8963/AK8963.h"
    #define mag Magnetometer
    AK8963 mag;
  #endif

#endif

//------------------------------ HUMIDITY OBJECTS ------------------------------//
#if (HUMIDITY)
  #include <DHT.h>
  DHT humiditySensor(DHT_PIN, DHTTYPE);
#endif

//--------------------------------------------------------- FILTERS OBJECTS ---------------------------------------------------------

#if (BARO_FILTERS)
  #if (KALMAN_MovingAverage_BARO)
    #include <SimpleKalmanFilter.h>
    SimpleKalmanFilter kalmanBaroAltitude(1, 1, 0.01);
  #else
    #include "src/lib/Filtros/Filtros.h"
    FiltroMediaMovel baroFilter1(FILTER_VECTOR_SIZE_BARO);
    FiltroMediaMovel baroFilter2(FILTER_VECTOR_SIZE_BARO);
    FiltroMediaMovel baroFilter3(FILTER_VECTOR_SIZE_BARO);
  #endif
#endif

#if (ACCEL_FILTERS)
  #if (KALMAN_MovingAverage_ACCEL)
    #include <SimpleKalmanFilter.h>
    SimpleKalmanFilter kalmanAccelX(2, 2, 0.01);
    SimpleKalmanFilter kalmanAccelY(2, 2, 0.01);
    SimpleKalmanFilter kalmanAccelZ(2, 2, 0.01);
  #else
    #include "src/lib/Filtros/Filtros.h"
    FiltroMediaMovel accelFilterX(FILTER_VECTOR_SIZE_ACCEL);
    FiltroMediaMovel accelFilterY(FILTER_VECTOR_SIZE_ACCEL);
    FiltroMediaMovel accelFilterZ(FILTER_VECTOR_SIZE_ACCEL);
  #endif
#endif

#if (GYRO_FILTERS)
  #if (KALMAN_MovingAverage_GYRO)
    #include <SimpleKalmanFilter.h>
    SimpleKalmanFilter kalmanGyroX(2, 2, 0.01);
    SimpleKalmanFilter kalmanGyroY(2, 2, 0.01);
    SimpleKalmanFilter kalmanGyroZ(2, 2, 0.01);
  #else
    #include "src/lib/Filtros/Filtros.h"
    FiltroMediaMovel gyroFilterX(FILTER_VECTOR_SIZE_GYRO);
    FiltroMediaMovel gyroFilterY(FILTER_VECTOR_SIZE_GYRO);
    FiltroMediaMovel gyroFilterZ(FILTER_VECTOR_SIZE_GYRO);
  #endif
#endif

#if (MAG_FILTERS)
  #if (KALMAN_MovingAverage_MAG)
    #include <SimpleKalmanFilter.h>
    SimpleKalmanFilter kalmanMagX(2, 2, 0.01);
    SimpleKalmanFilter kalmanMagY(2, 2, 0.01);
    SimpleKalmanFilter kalmanMagZ(2, 2, 0.01);
  #else
    #include "src/lib/Filtros/Filtros.h"
    FiltroMediaMovel magFilterX(FILTER_VECTOR_SIZE_MAG);
    FiltroMediaMovel magFilterY(FILTER_VECTOR_SIZE_MAG);
    FiltroMediaMovel magFilterZ(FILTER_VECTOR_SIZE_MAG);
  #endif
#endif

//------------------------------ RESTART PROTECTION MODE OBJECT ------------------------------//
#if (RESTART_PROTECTION)
  #if defined(PLACA_ESSENTIALS_V1) || defined(PLACA_MAIN_PTH) || defined(PLACA_MAIN_SMD) || defined(PLACA_MAIN_IC)
    #include <Preferences.h>
    Preferences prefs;
  #endif
#endif

//------------------------------ LORA MODE OBJECT ------------------------------//

#if (LORA_MODE)

  #if defined(PLACA_MAIN_OCTA_PTH)
    HardwareSerial& LoRa = Serial3;
  #elif defined(ARDUINO_ARCH_ESP32)
    HardwareSerial LoRa(1);
  #else
    #error "LoRa error on object declaration. Please check your board selection in the config."
  #endif
  
#if (LORA_MANAGER)

#if (LORA_FREQUENCY_MODE == 900)
  #define FREQUENCY_900
  #define LORA_MAX_CHANNEL 69
#elif (LORA_FREQUENCY_MODE == 433)
  #define FREQUENCY_433
  #define LORA_MAX_CHANNEL 0
  #error "433Mhz Not implemented yet."
#else
  #error "Invalid LoRa Frequency. Use 900 or 433 MHz."
#endif

#if (LORA_MANAGER_E32 && LORA_MANAGER_E22)
  #error "Multiple LoRa Managers defined. Define only one of LORA_MANAGER_E32 or LORA_MANAGER_E22."
#endif

#if (LORA_MANAGER_E32)
  #include <LoRa_E32.h>
#elif (LORA_MANAGER_E22)
  #include <LoRa_E22.h>
#else
  #error "No LoRa Manager defined. Define LORA_MANAGER_E32 or LORA_MANAGER_E22."
#endif

  #include "src/lib/LoRaManager/LoRaManager.h"

  bool changingFrequency = false;
  unsigned long lastLoraCommandTime = 0;
  int channel;
  unsigned long addh;
  unsigned long addl;

  LoRa_E32 e32Module(
    &LoRa,
    AUX_LORA_PIN,
    M0_LORA_PIN,
    M1_LORA_PIN
  );

  LoRaManager loraManager(LoRa);
#endif
#endif

//------------------------------ GPS MODE OBJECT ------------------------------//
#if (GPS_MODE)
  #include <TinyGPSPlus.h>
  TinyGPSPlus gps;

  #if defined(PLACA_MAIN_OCTA_PTH)
    HardwareSerial& GPS = Serial1;
  #elif defined(ARDUINO_ARCH_ESP32)
    HardwareSerial GPS(2);
  #else
    HardwareSerial GPS(2);
  #endif
#endif

//------------------------------ STORAGE MODE OBJECT ------------------------------//

#if (STORAGE_MODE)
  #if (SD_CARD)
    #include <SD.h>
    #include <SPI.h>

    #if defined(CONFIG_IDF_TARGET_ESP32S3)
      SPIClass SPI_SD(FSPI);
    #endif

    File logFile;
  #endif

  #if (EEPROM)
    #include <EEPROM.h>
  #endif

  char baseNameSD[] = FILE_NAME;
  char nameSD[13];
#endif

//------------------------------ 3D ORIENTATION OBJECT ------------------------------//
#if (ENABLE_3D)
  #include <MadgwickAHRS.h>
  Madgwick orientation;
#endif

//------------------------------ APOGEE AND PARACHUTE OBJECTS ------------------------------//
#if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
  #include "src/lib/Apogeu/Apogeu.h"
  Apogeu apogeeBaro(10, APOGEE_DETECTION_THRESHOLD);
#endif

#if (PARACHUTE)
  #include "src/lib/Paraquedas/Paraquedas.h"

  #if (Parachute_FirstNormalEvent)
    Paraquedas parachuteFirstNormal(
      ACTIVE_GPIO_TIME,
      FIRST_NORMAL_EVENT_DELAY,
      IGN_1,
      HEAL_1,
      ACTIVATION_HEIGHT_FIRST_NORMAL_EVENT
    );
  #endif

  #if (Parachute_FirstBackupEvent)
    Paraquedas parachuteFirstBackup(
      ACTIVE_GPIO_TIME,
      FIRST_BACKUP_EVENT_DELAY,
      IGN_2,
      HEAL_2,
      ACTIVATION_HEIGHT_FIRST_BACKUP_EVENT
    );
  #endif

  #if (Parachute_SecondNormalEvent)
    Paraquedas parachuteSecondNormal(
      ACTIVE_GPIO_TIME,
      SECOND_NORMAL_EVENT_DELAY,
      IGN_3,
      HEAL_3,
      ACTIVATION_HEIGHT_SECOND_NORMAL_EVENT
    );
  #endif

  #if (Parachute_SecondBackupEvent)
    Paraquedas parachuteSecondBackup(
      ACTIVE_GPIO_TIME,
      SECOND_BACKUP_EVENT_DELAY,
      IGN_4,
      HEAL_4,
      ACTIVATION_HEIGHT_SECOND_BACKUP_EVENT
    );
  #endif
#endif

//------------------------------ URD APP OBJECT ------------------------------//
#if (URD_APP_SERIAL)
  #include "src/lib/UrdApp/urd_app.h"
  UrdApp urdApp(SERIAL_BAUD_RATE);
#endif

//=================================================================================== GLOBAL STRUCTS ===================================================================================

struct SensorData {
  uint32_t timestampMs;
  uint32_t frequencyHz;

#if (BARO)
  bool  barometerAvailable;
  float barometerTemperatureC;
  float barometerPressurePa;
  float barometerAltitudeM;
  float barometerFilteredAltitudeM;
  float humidityPercent;
#endif

#if (URD_APP_SIMULATION_MODE)
  bool  simulationAvailable;
  float simulationPressurePa;
  float simulationAltitudeM;
#endif

#if (ACCEL)
  bool  accelerometerAvailable;
  float accelX;
  float accelY;
  float accelZ;
  float accelFilteredX;
  float accelFilteredY;
  float accelFilteredZ;
#endif

#if (GYRO)
  bool  gyroscopeAvailable;
  float gyroX;
  float gyroY;
  float gyroZ;
  float gyroFilteredX;
  float gyroFilteredY;
  float gyroFilteredZ;
#endif

#if (MAG)
  bool  magnetometerAvailable;
  float magX;
  float magY;
  float magZ;
  float magFilteredX;
  float magFilteredY;
  float magFilteredZ;
#endif

#if (HUMIDITY)
  bool  humidityAvailable;
  float humidityTemperatureC;
  float humidityRelativePercent;
#endif

#if (GPS_MODE)
  bool  gpsAvailable;
  double gpsLatitudeDeg;
  double gpsLongitudeDeg;
  float gpsAltitudeM;
  float gpsSpeedKmph;
  uint32_t gpsSatellites;
  float gpsHdop;
#endif

#if (BATTERY_MONITOR)
  bool  batteryState;
  float batteryVoltage;
  float batteryPercent;
#endif

#if (ENABLE_3D)
  bool  orientationAvailable;
  bool  orientationConverged;
  bool  orientationOffsetReady;
  float rollDeg;
  float pitchDeg;
  float yawDeg;
#endif

};

struct FlightEventsData {
  uint32_t timestampMs;
  float inputAltitudeM;

#if (WUF_PROTECTION)
  bool wufLockActive;
#endif

#if (TRANSONIC_PROTECTION)
  bool transonicLockActive;
#endif

#if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
  bool  descending;
  bool  apogeeDetected;
  float apogeeAltitudeM;
  uint32_t apogeeTimeMs;
  float maxTrackedAltitudeM;
#endif

#if (PARACHUTE)
  #if (Parachute_FirstNormalEvent)
    uint8_t parachuteFirstNormalData;
    bool    parachuteFirstNormalContinuity;
    float   parachuteFirstNormalAltitudeM;
    uint32_t parachuteFirstNormalTimeMs;
  #endif

  #if (Parachute_FirstBackupEvent)
    uint8_t parachuteFirstBackupData;
    bool    parachuteFirstBackupContinuity;
    float   parachuteFirstBackupAltitudeM;
    uint32_t parachuteFirstBackupTimeMs;
  #endif

  #if (Parachute_SecondNormalEvent)
    uint8_t parachuteSecondNormalData;
    bool    parachuteSecondNormalContinuity;
    float   parachuteSecondNormalAltitudeM;
    uint32_t parachuteSecondNormalTimeMs;
  #endif

  #if (Parachute_SecondBackupEvent)
    uint8_t parachuteSecondBackupData;
    bool    parachuteSecondBackupContinuity;
    float   parachuteSecondBackupAltitudeM;
    uint32_t parachuteSecondBackupTimeMs;
  #endif
#endif
};

struct SystemStates {
  bool startupDone;

#if (PARACHUTE)
  bool parachuteFirstNormalHealth;
  bool parachuteFirstBackupHealth;
  bool parachuteSecondNormalHealth;
  bool parachuteSecondBackupHealth;
#endif

#if (BARO)
  bool barometerEnabled = BARO;
  bool barometerInitOk;
  bool barometerRuntimeOk;
#endif

#if (URD_APP_SIMULATION_MODE)
  bool simulationEnabled = URD_APP_SIMULATION_MODE;
  bool simulationInitOk;
  bool simulationRuntimeOk;
#endif

#if (COMMUNICATION_ON)
  bool storageEnabled = STORAGE_MODE;
  bool storageInitOk;
  bool storageRuntimeOk;
#endif

#if (ACCEL)
  bool accelerometerEnabled = ACCEL;
  bool accelerometerInitOk;
  bool accelerometerRuntimeOk;
#endif

#if (GYRO)
  bool gyroscopeEnabled = GYRO;
  bool gyroscopeInitOk;
  bool gyroscopeRuntimeOk;
#endif

#if (MAG)
  bool magnetometerEnabled = MAG;
  bool magnetometerInitOk;
  bool magnetometerRuntimeOk;
#endif

#if (HUMIDITY)
  bool humidityEnabled = HUMIDITY;
  bool humidityInitOk;
  bool humidityRuntimeOk;
#endif

#if (GPS_MODE)
  bool gpsEnabled = GPS_MODE;
  bool gpsInitOk;
  bool gpsRuntimeOk;
#endif

#if (BATTERY_MONITOR)
  bool batteryMonitorEnabled = BATTERY_MONITOR;
  bool batteryInitOk;
  bool batteryRuntimeOk;
#endif

#if (ENABLE_3D)
  bool orientationEnabled = ENABLE_3D;
  bool orientationInitOk;
  bool orientationRuntimeOk;
#endif
};

enum FlightPhase {
  FLIGHT_PHASE_INITIALIZATION = 0,
  FLIGHT_PHASE_PRE_FLIGHT,
  FLIGHT_PHASE_BOOST,
  FLIGHT_PHASE_COAST,
  FLIGHT_PHASE_FIRST_EVENT,
  FLIGHT_PHASE_SECOND_EVENT,
  FLIGHT_PHASE_LANDED
};

struct FlightState {
  FlightPhase phase;

  bool wufPassed;
  bool boostDetected;
  bool firstEventTriggered;
  bool secondEventTriggered;
  bool landedDetected;

  float currentAltitudeM;
  float maxAltitudeM;
  float resultantAccelerationMs2;

  uint32_t phaseTimestampMs;
  uint32_t lastLandingCheckMs;
  float landingReferenceAltitudeM;
};

//=================================================================================== GLOBAL VARIABLES ===================================================================================

FlightState gFlightState = {};
SemaphoreHandle_t gFlightStateMutex = nullptr;

SystemStates gSystemStates = {};
SemaphoreHandle_t gSystemStatesMutex = nullptr;

#if (SENSORS_ON)
SensorData gSensorData = {};
SemaphoreHandle_t gSensorMutex = nullptr;
static uint32_t lastTimestampMs = 0;
#endif

#if (URD_APP_SERIAL)
// bool inicioUrdApp = false;
static uint32_t lastUrdAppOutputMs = 0;
#endif

#if (BARO || URD_APP_SIMULATION_MODE)
float startupPressurePa = 0.0f;
float startupAltitudeM = 0.0f;
#endif

#if (RESTART_PROTECTION)
bool flightActive = false;
#endif

#if (COMMUNICATION_ON)
TaskHandle_t hTaskCommunication = nullptr;

static bool sdWriteOk = false;
static bool sdInitOk = false;
static bool sdDelayProtectionWhenErr = false;

static uint32_t lastSerialOutputMs = 0;
static uint32_t lastSdOutputMs = 0;
static uint32_t lastSdDelayOutputMs = 0;
static uint32_t lastLoRaOutputMs = 0;

static bool serialHeaderPrinted = false;
static bool loraHeaderPrinted = false;
static bool sdHeaderPrinted = false;
#endif

#if (BUZZER_INDICATOR)
TaskHandle_t hTaskBuzzer = nullptr;
#endif

#if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
FlightEventsData gFlightEventsData = {};
SemaphoreHandle_t gFlightEventsMutex = nullptr;
TaskHandle_t hTaskFlightEvents = nullptr;
#endif

#if (WUF_PROTECTION)
static bool waitUntilFlightActive = true;
#endif

#if (TRANSONIC_PROTECTION)
static bool transonicProtectionActive = true;
static uint32_t transonicStartTimeMs = 0;
#endif

#if (ENABLE_3D)
static uint32_t orientationStartTimeMs = 0;
static bool orientationWarmupDone = false;
static bool orientationOffsetsReady = false;

static uint16_t orientationOffsetSampleCount = 0;

static float rollOffsetSumDeg = 0.0f;
static float pitchOffsetSumDeg = 0.0f;
static float yawOffsetSumDeg = 0.0f;

static float rollOffsetDeg = 0.0f;
static float pitchOffsetDeg = 0.0f;
static float yawOffsetDeg = 0.0f;
#endif

//=================================================================================== FUNCTIONS ===================================================================================
//--------------------------------------------------------- AUXILIARY FUNCTIONS ---------------------------------------------------

void copyFlightState(FlightState &dst) {
  if (xSemaphoreTake(gFlightStateMutex, portMAX_DELAY) == pdTRUE) {
    dst = gFlightState;
    xSemaphoreGive(gFlightStateMutex);
  }
}

void updateFlightState(const FlightState &src) {
  if (xSemaphoreTake(gFlightStateMutex, portMAX_DELAY) == pdTRUE) {
    gFlightState = src;
    xSemaphoreGive(gFlightStateMutex);
  }
}

void copySystemStates(SystemStates &dst) {
  if (xSemaphoreTake(gSystemStatesMutex, portMAX_DELAY) == pdTRUE) {
    dst = gSystemStates;
    xSemaphoreGive(gSystemStatesMutex);
  }
}

void updateSystemStates(const SystemStates &src) {
  if (xSemaphoreTake(gSystemStatesMutex, portMAX_DELAY) == pdTRUE) {
    gSystemStates = src;
    xSemaphoreGive(gSystemStatesMutex);
  }
}

#if (BARO || URD_APP_SIMULATION_MODE)
float pressureToHeight(float pressurePa) {
  return 44330.0f * (1.0f - powf(pressurePa / 101325.0f, 1.0f / 5.25588f));
}
#endif

void copySensorData(SensorData &dst) {
  if (xSemaphoreTake(gSensorMutex, portMAX_DELAY) == pdTRUE) {
    dst = gSensorData;
    xSemaphoreGive(gSensorMutex);
  }
}

void updateSensorData(const SensorData &src) {
  if (xSemaphoreTake(gSensorMutex, portMAX_DELAY) == pdTRUE) {
    gSensorData = src;
    xSemaphoreGive(gSensorMutex);
  }
}

#if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
void copyFlightEventsData(FlightEventsData &dst) {
  if (xSemaphoreTake(gFlightEventsMutex, portMAX_DELAY) == pdTRUE) {
    dst = gFlightEventsData;
    xSemaphoreGive(gFlightEventsMutex);
  }
}

void updateFlightEventsData(const FlightEventsData &src) {
  if (xSemaphoreTake(gFlightEventsMutex, portMAX_DELAY) == pdTRUE) {
    gFlightEventsData = src;
    xSemaphoreGive(gFlightEventsMutex);
  }
}
#endif


#if (ENABLE_3D)
float normalizeAngle180(float angleDeg) {
  while (angleDeg > 180.0f) angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}
#endif

//--------------------------------------------------------- URD APP FUNCTIONS ---------------------------------------------------------
#if (URD_APP_SERIAL)

// void resetar() {
//   ESP.restart();
// }

// void startUrdAppSerial() {
//   #if (BUZZER_INDICATOR)
//   playDot();
//   playDot();
//   playDot();
//   delay(2000);
//   #endif

//   bool readyReceived = false;

//   Serial.println("Starting URD App Serial Communication...");
//   while (!inicioUrdApp) {
//     #if (BUZZER_INDICATOR)
//     if (!readyReceived) {
//       playDot();
//     }
//     #endif
//     if (Serial.available()) {
//       String msg = Serial.readStringUntil('\n');
//       msg.trim();

//       if (msg == "READY") {
//         Serial.println("OK");
//         readyReceived = true;
//       }

//       else if (msg == "GPS_COORDS") {
//         Serial.println("~\t~");
//         inicioUrdApp = true;
//         Serial.println("---Flight Computer RTOS---");
//       }
//       #if (URD_APP_SIMULATION_MODE)
//       else if (msg == "SIMULATION") {
//         Serial.println("STARTED");
//         inicioUrdApp = true;
//       }

//       else if (msg == "RST") {
//         resetar();
//       }
//     }

//     delay(10);
//   }
// }

#endif

//--------------------------------------------------------- BUZZER FUNCTIONS ---------------------------------------------------------
#if (BUZZER_INDICATOR)
void buzzerOn(int frequencyHz = 2000) {
  #if (BUZZER_PASSIVE_MODE)
    tone(BUZZ_PIN, frequencyHz);
  #else
    digitalWrite(BUZZ_PIN, BUZZ_ON);
  #endif
}

void buzzerOff() {
  #if (BUZZER_PASSIVE_MODE)
    noTone(BUZZ_PIN);
  #else
    digitalWrite(BUZZ_PIN, !BUZZ_ON);
  #endif
}

void playDot() {
  buzzerOn();
  vTaskDelay(pdMS_TO_TICKS(BUZZER_DOT_TIME_MS));
  buzzerOff();
  vTaskDelay(pdMS_TO_TICKS(BUZZER_SYMBOL_PAUSE_MS));
}

void playDash() {
  buzzerOn();
  vTaskDelay(pdMS_TO_TICKS(BUZZER_DASH_TIME_MS));
  buzzerOff();
  vTaskDelay(pdMS_TO_TICKS(BUZZER_SYMBOL_PAUSE_MS));
}

void playMorseCode(const char* code) {
  for (size_t i = 0; code[i] != '\0'; i++) {
    if (code[i] == '.') {
      playDot();
    } else if (code[i] == '-') {
      playDash();
    } else if (code[i] == ' ') {
      vTaskDelay(pdMS_TO_TICKS(BUZZER_LETTER_MORSE_PAUSE_MS)); // Pause between letters
    }
  }
  vTaskDelay(pdMS_TO_TICKS(BUZZER_WORD_MORSE_PAUSE_MS)); // Pause after the whole message
}



void playMaxAltitudeAsDots(float maxAltitude)
{
  if (maxAltitude < 0.0f) {
    return;
  }

  uint32_t altitudeMeters = (uint32_t)(maxAltitude);

  if (altitudeMeters > 9999) {
    return;
  }

  uint8_t thousands = (altitudeMeters / 1000) % 10;
  uint8_t hundreds  = (altitudeMeters / 100)  % 10;
  uint8_t tens      = (altitudeMeters / 10)   % 10;

  for (uint8_t i = 0; i < thousands; i++) {
    playDot();
  }

  vTaskDelay(pdMS_TO_TICKS(MAX_ALTITUDE_DIGIT_GAP_MS));

  for (uint8_t i = 0; i < hundreds; i++) {
    playDot();
  }

  vTaskDelay(pdMS_TO_TICKS(MAX_ALTITUDE_DIGIT_GAP_MS));

  for (uint8_t i = 0; i < tens; i++) {
    playDot();
  }

  vTaskDelay(pdMS_TO_TICKS(MAX_ALTITUDE_BEEP_GAP_MS));

}


void playLandingSiren(uint16_t baseFreqHz)
{
  enum SirenType : uint8_t {
    SIREN_WAIL = 0,
    SIREN_YELP,
    SIREN_HI_LO,
    SIREN_FAST_PULSE
  };

  enum {
    SIREN_COUNT = 4
  };

  static bool randomStarted = false;
  static uint8_t sirenOrder[SIREN_COUNT] = {
    SIREN_WAIL,
    SIREN_YELP,
    SIREN_HI_LO,
    SIREN_FAST_PULSE
  };

  static uint8_t currentSirenIndex = SIREN_COUNT;
  static uint8_t lastPlayedSiren = 255;

  if (!randomStarted) {
#if defined(ARDUINO_ARCH_ESP32)
    randomSeed((uint32_t)esp_random());
#else
    randomSeed((uint32_t)micros());
#endif

    randomStarted = true;
  }

  if (currentSirenIndex >= SIREN_COUNT) {
    sirenOrder[0] = SIREN_WAIL;
    sirenOrder[1] = SIREN_YELP;
    sirenOrder[2] = SIREN_HI_LO;
    sirenOrder[3] = SIREN_FAST_PULSE;

    for (int8_t i = SIREN_COUNT - 1; i > 0; i--) {
      uint8_t randomIndex = random(0, i + 1);

      uint8_t temp = sirenOrder[i];
      sirenOrder[i] = sirenOrder[randomIndex];
      sirenOrder[randomIndex] = temp;
    }

    // Evita começar o novo ciclo com a mesma sirene que acabou de tocar
    if (sirenOrder[0] == lastPlayedSiren) {
      uint8_t temp = sirenOrder[0];
      sirenOrder[0] = sirenOrder[1];
      sirenOrder[1] = temp;
    }

    currentSirenIndex = 0;
  }

  uint8_t selectedSiren = sirenOrder[currentSirenIndex];
  currentSirenIndex++;

  switch (selectedSiren) {
    case SIREN_WAIL:
      playSirenWail(baseFreqHz);
      break;

    case SIREN_YELP:
      playSirenYelp(baseFreqHz);
      break;

    case SIREN_HI_LO:
      playSirenHiLo(baseFreqHz);
      break;

    case SIREN_FAST_PULSE:
      playSirenFastPulse(baseFreqHz);
      break;

    default:
      playSirenFastPulse(baseFreqHz);
      break;
  }

  lastPlayedSiren = selectedSiren;
}

void playSirenWail(uint16_t baseFreqHz)
{
#if (BUZZER_PASSIVE_MODE)

  uint16_t minFreqHz = baseFreqHz / 2;
  uint16_t maxFreqHz = baseFreqHz + (baseFreqHz / 2);

  if (minFreqHz < 300) {
    minFreqHz = 300;
  }

  if (maxFreqHz > 5000) {
    maxFreqHz = 5000;
  }

  for (uint16_t freq = minFreqHz; freq <= maxFreqHz; freq += 50) {
    buzzerOn(freq);
    vTaskDelay(pdMS_TO_TICKS(25));
  }

  for (int32_t freq = maxFreqHz; freq >= minFreqHz; freq -= 50) {
    buzzerOn((uint16_t)freq);
    vTaskDelay(pdMS_TO_TICKS(25));
  }

  buzzerOff();
  vTaskDelay(pdMS_TO_TICKS(SIREN_PAUSE_MS));

#else

  for (uint8_t i = 0; i < 3; i++) {
    buzzerOn(baseFreqHz);
    vTaskDelay(pdMS_TO_TICKS(400));

    buzzerOff();
    vTaskDelay(pdMS_TO_TICKS(180));
  }

#endif
}

void playSirenYelp(uint16_t baseFreqHz)
{
#if (BUZZER_PASSIVE_MODE)

  uint16_t minFreqHz = baseFreqHz / 2;
  uint16_t maxFreqHz = baseFreqHz + (baseFreqHz / 2);

  if (minFreqHz < 500) {
    minFreqHz = 500;
  }

  if (maxFreqHz > 5000) {
    maxFreqHz = 5000;
  }

  for (uint8_t cycle = 0; cycle < 3; cycle++) {
    for (uint16_t freq = minFreqHz; freq <= maxFreqHz; freq += 150) {
      buzzerOn(freq);
      vTaskDelay(pdMS_TO_TICKS(12));
    }

    for (int32_t freq = maxFreqHz; freq >= minFreqHz; freq -= 150) {
      buzzerOn((uint16_t)freq);
      vTaskDelay(pdMS_TO_TICKS(12));
    }
  }

  buzzerOff();
  vTaskDelay(pdMS_TO_TICKS(SIREN_PAUSE_MS));

#else

  for (uint8_t i = 0; i < 10; i++) {
    buzzerOn(baseFreqHz);
    vTaskDelay(pdMS_TO_TICKS(90));

    buzzerOff();
    vTaskDelay(pdMS_TO_TICKS(60));
  }

#endif
}

void playSirenHiLo(uint16_t baseFreqHz)
{
#if (BUZZER_PASSIVE_MODE)

  uint16_t lowFreqHz  = baseFreqHz;
  uint16_t highFreqHz = baseFreqHz + 700;

  if (highFreqHz > 5000) {
    highFreqHz = 5000;
  }

  for (uint8_t i = 0; i < 4; i++) {
    buzzerOn(highFreqHz);
    vTaskDelay(pdMS_TO_TICKS(300));

    buzzerOn(lowFreqHz);
    vTaskDelay(pdMS_TO_TICKS(300));
  }

  buzzerOff();
  vTaskDelay(pdMS_TO_TICKS(SIREN_PAUSE_MS));

#else

  for (uint8_t i = 0; i < 4; i++) {
    buzzerOn(baseFreqHz);
    vTaskDelay(pdMS_TO_TICKS(300));

    buzzerOff();
    vTaskDelay(pdMS_TO_TICKS(120));

    buzzerOn(baseFreqHz);
    vTaskDelay(pdMS_TO_TICKS(160));

    buzzerOff();
    vTaskDelay(pdMS_TO_TICKS(220));
  }

#endif
}

void playSirenFastPulse(uint16_t baseFreqHz)
{
#if BUZZER_PASSIVE

  uint16_t pulseFreqHz = baseFreqHz + 500;

  if (pulseFreqHz > 5000) {
    pulseFreqHz = 5000;
  }

  for (uint8_t i = 0; i < 12; i++) {
    buzzerOn(pulseFreqHz);
    vTaskDelay(pdMS_TO_TICKS(70));

    buzzerOff();
    vTaskDelay(pdMS_TO_TICKS(70));
  }

  vTaskDelay(pdMS_TO_TICKS(SIREN_PAUSE_MS));

#else

  for (uint8_t i = 0; i < 12; i++) {
    buzzerOn(baseFreqHz);
    vTaskDelay(pdMS_TO_TICKS(70));

    buzzerOff();
    vTaskDelay(pdMS_TO_TICKS(70));
  }

  vTaskDelay(pdMS_TO_TICKS(SIREN_PAUSE_MS));

#endif
}

#endif
//--------------------------------------------------------- COMMUNICATION FUNCTIONS ---------------------------------------------------------
#if (COMMUNICATION_ON)
String buildDataHeader() {
  String header = "";

  header += "time_s\t";
  header += "freq_hz\t";
  header += "flight_phase\t";

#if (BARO)
  header += "alt_m\t";
  header += "alt_max_m\t";
#endif

#if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
  header += "descending\t";
  header += "apogee_alt_m\t";
  header += "apogee_time_s\t";
#endif

#if (PARACHUTE)
  #if (Parachute_FirstNormalEvent)
    header += "p1_data\t";
    #if (HEAL)
      header += "p1_heal\t";
    #endif
    header += "p1_alt_m\t";
    header += "p1_time_s\t";
  #endif

  #if (Parachute_FirstBackupEvent)
    header += "p2_data\t";
    #if (HEAL)
      header += "p2_heal\t";
    #endif
    header += "p2_alt_m\t";
    header += "p2_time_s\t";
  #endif

  #if (Parachute_SecondNormalEvent)
    header += "p3_data\t";
    #if (HEAL)
      header += "p3_heal\t";
    #endif
    header += "p3_alt_m\t";
    header += "p3_time_s\t";
  #endif

  #if (Parachute_SecondBackupEvent)
    header += "p4_data\t";
    #if (HEAL)
      header += "p4_heal\t";
    #endif
    header += "p4_alt_m\t";
    header += "p4_time_s\t";
  #endif
#endif

#if (GPS_MODE)
  header += "lat_deg\t";
  header += "lon_deg\t";
  header += "gps_alt_m\t";
  header += "gps_speed_kmph\t";
  header += "gps_sat\t";
  header += "gps_hdop\t";
#endif

#if (BARO)
  header += "baro_temp_c\t";
  header += "baro_press_pa\t";
#endif

#if (ACCEL)
  header += "ax\tay\taz\t";
#endif

#if (GYRO)
  header += "gx\tgy\tgz\t";
#endif

#if (MAG)
  header += "mx\tmy\tmz\t";
#endif

#if (HUMIDITY)
  header += "hum_temp_c\t";
  header += "hum_rh\t";
#endif

#if (BATTERY_MONITOR)
  header += "battery_state\t";
#endif

#if (STORAGE_MODE && SD_CARD)
  header += "sd_ok";
#endif

#if (ENABLE_3D)
  header += "roll_deg\t";
  header += "pitch_deg\t";
  header += "yaw_deg\t";
#endif

  return header;
}

#if (LORA_MODE)
String buildLoraHeader() {
  return "L\tT\tPHASE\tLAT\tLON\th\tmin\tHDOP\tALT\tAPGH\tAPGT\tPQD1234\tROLL\tPITCH\tYAW";
}
#endif

#if (NORMAL_SERIAL_PRINT_MODE || STORAGE_MODE)
String buildDataString(const SensorData &sensorData, const FlightEventsData &flightData) {
  FlightState localFlightState;
  copyFlightState(localFlightState);

  String dataString = "";

  dataString += String(sensorData.timestampMs / 1000.0f, 3) + '\t';
  dataString += String(sensorData.frequencyHz) + '\t';
  dataString += String(localFlightState.phase) + '\t';

#if (BARO)
  dataString += String(sensorData.barometerFilteredAltitudeM, 2) + '\t';
  dataString += String(flightData.maxTrackedAltitudeM, 2) + '\t';
#endif

#if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
  dataString += String(flightData.descending ? 1 : 0) + '\t';
  dataString += String(flightData.apogeeAltitudeM, 2) + '\t';
  dataString += String(flightData.apogeeTimeMs / 1000.0f, 2) + '\t';
#endif

#if (PARACHUTE)
  #if (Parachute_FirstNormalEvent)
    dataString += String(flightData.parachuteFirstNormalData) + '\t';
    #if (HEAL)
      dataString += String(flightData.parachuteFirstNormalContinuity ? 1 : 0) + '\t';
    #endif
    dataString += String(flightData.parachuteFirstNormalAltitudeM, 2) + '\t';
    dataString += String(flightData.parachuteFirstNormalTimeMs / 1000.0f, 2) + '\t';
  #endif

  #if (Parachute_FirstBackupEvent)
    dataString += String(flightData.parachuteFirstBackupData) + '\t';
    #if (HEAL)
      dataString += String(flightData.parachuteFirstBackupContinuity ? 1 : 0) + '\t';
    #endif
    dataString += String(flightData.parachuteFirstBackupAltitudeM, 2) + '\t';
    dataString += String(flightData.parachuteFirstBackupTimeMs / 1000.0f, 2) + '\t';
  #endif

  #if (Parachute_SecondNormalEvent)
    dataString += String(flightData.parachuteSecondNormalData) + '\t';
    #if (HEAL)
      dataString += String(flightData.parachuteSecondNormalContinuity ? 1 : 0) + '\t';
    #endif
    dataString += String(flightData.parachuteSecondNormalAltitudeM, 2) + '\t';
    dataString += String(flightData.parachuteSecondNormalTimeMs / 1000.0f, 2) + '\t';
  #endif

  #if (Parachute_SecondBackupEvent)
    dataString += String(flightData.parachuteSecondBackupData) + '\t';
    #if (HEAL)
      dataString += String(flightData.parachuteSecondBackupContinuity ? 1 : 0) + '\t';
    #endif
    dataString += String(flightData.parachuteSecondBackupAltitudeM, 2) + '\t';
    dataString += String(flightData.parachuteSecondBackupTimeMs / 1000.0f, 2) + '\t';
  #endif
#endif

#if (GPS_MODE)
  if (sensorData.gpsAvailable) {
    dataString += String(sensorData.gpsLatitudeDeg, 6) + '\t';
    dataString += String(sensorData.gpsLongitudeDeg, 6) + '\t';
    dataString += String(sensorData.gpsAltitudeM, 2) + '\t';
    dataString += String(sensorData.gpsSpeedKmph, 2) + '\t';
    dataString += String(sensorData.gpsSatellites) + '\t';
    dataString += String(sensorData.gpsHdop, 1) + '\t';
  } else {
    dataString += "~\t~\t~\t~\t~\t~\t";
  }
#endif

#if (BARO)
  dataString += String(sensorData.barometerTemperatureC, 2) + '\t';
  dataString += String(sensorData.barometerPressurePa, 2) + '\t';
#endif

#if (ACCEL)
#if (AUTO_CALIBRATION_ACCEL)
  dataString += String(sensorData.accelFilteredX, 3) + '\t';
  dataString += String(sensorData.accelFilteredY, 3) + '\t';
  dataString += String(sensorData.accelFilteredZ, 3) + '\t';
#else
  dataString += String(sensorData.accelX, 3) + '\t';
  dataString += String(sensorData.accelY, 3) + '\t';
  dataString += String(sensorData.accelZ, 3) + '\t';
#endif
#endif

#if (GYRO)
#if (AUTO_CALIBRATION_GYRO)
  dataString += String(sensorData.gyroFilteredX, 3) + '\t';
  dataString += String(sensorData.gyroFilteredY, 3) + '\t';
  dataString += String(sensorData.gyroFilteredZ, 3) + '\t';
#else
  dataString += String(sensorData.gyroX, 3) + '\t';
  dataString += String(sensorData.gyroY, 3) + '\t';
  dataString += String(sensorData.gyroZ, 3) + '\t';
#endif
#endif

#if (MAG)
#if (AUTO_CALIBRATION_MAG)
  dataString += String(sensorData.magFilteredX, 3) + '\t';
  dataString += String(sensorData.magFilteredY, 3) + '\t';
  dataString += String(sensorData.magFilteredZ, 3) + '\t';
#else
  dataString += String(sensorData.magX, 3) + '\t';
  dataString += String(sensorData.magY, 3) + '\t';
  dataString += String(sensorData.magZ, 3) + '\t';
#endif
#endif

#if (HUMIDITY)
  dataString += String(sensorData.humidityTemperatureC, 2) + '\t';
  dataString += String(sensorData.humidityRelativePercent, 2) + '\t';
#endif

#if (BATTERY_MONITOR)
  dataString += String(sensorData.batteryState, 2) + '\t';
#endif

#if (STORAGE_MODE && SD_CARD)
  dataString += String(sdWriteOk ? 1 : 0) + '\t';
#endif

#if (ENABLE_3D)
  if (sensorData.orientationAvailable) {
    dataString += String(sensorData.rollDeg, 2) + '\t';
    dataString += String(sensorData.pitchDeg, 2) + '\t';
    dataString += String(sensorData.yawDeg, 2) + '\t';
  } else {
    dataString += "NAN\tNAN\tNAN\t";
  }
#endif

  return dataString;
}
#endif

#if (URD_APP_SERIAL)
  #if (URD_APP_SIMULATION_MODE || URD_APP_GS_FLIGHT_COMPUTER_SIMULATION_MODE)
  String buildDataStringUrdApp(const SensorData &sensorData, const FlightEventsData &flightData) {
    FlightState localFlightState;
    copyFlightState(localFlightState);

    String dataString = "";

    uint32_t linha = 0;

    int flightPhase = localFlightState.phase;

    float t = sensorData.timestampMs / 1000.0f;

    float latitude = 0.0f;
    float longitude = 0.0f;

    uint8_t hora_value = 0;
    uint8_t minuto_value = 0;

    float precisao_value = 0.0f;

    float altitude = 0.0f;
    uint8_t sd_value = 0;

    float apogeu_h = 0.0f;
    float apogeu_t = 0.0f;
    bool apogeu_detectado = false;

    float pq_dn_altitude = 0.0f;
    float pq_db_altitude = 0.0f;
    float pq_mn_altitude = 0.0f;
    float pq_mb_altitude = 0.0f;

    float temp_value = 0.0f;

    float euler_roll = 0.0f;
    float euler_pitch = 0.0f;
    float euler_yaw = 0.0f;


  #if (GPS_MODE)
    if (sensorData.gpsAvailable) {
      latitude = sensorData.gpsLatitudeDeg;
      longitude = sensorData.gpsLongitudeDeg;

      precisao_value = sensorData.gpsHdop;
    }
  #endif

  #if (BARO)
    altitude = sensorData.barometerFilteredAltitudeM;
    temp_value = sensorData.barometerTemperatureC;
  #endif

  #if (URD_APP_SIMULATION_MODE)
    altitude = sensorData.simulationAltitudeM;
    temp_value = 25.0f;
  #endif

  #if (STORAGE_MODE && SD_CARD)
    sd_value = sdWriteOk ? 1 : 0;
  #endif

  #if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
    apogeu_h = flightData.apogeeAltitudeM;
    apogeu_t = flightData.apogeeTimeMs / 1000.0f;
  #endif

  #if (PARACHUTE)

    #if (Parachute_FirstNormalEvent)
      pq_dn_altitude = flightData.parachuteFirstNormalAltitudeM;
    #endif

    #if (Parachute_FirstBackupEvent)
      pq_db_altitude = flightData.parachuteFirstBackupAltitudeM;
    #endif

    #if (Parachute_SecondNormalEvent)
      pq_mn_altitude = flightData.parachuteSecondNormalAltitudeM;
    #endif

    #if (Parachute_SecondBackupEvent)
      pq_mb_altitude = flightData.parachuteSecondBackupAltitudeM;
    #endif

  #endif

  #if (ENABLE_3D)
    if (sensorData.orientationAvailable) {
      euler_roll = sensorData.rollDeg;
      euler_pitch = sensorData.pitchDeg;
      euler_yaw = sensorData.yawDeg;
    }
  #endif

    dataString += "L" + String(linha) + "\t";
    dataString += "T" + String(t, 2) + "\t";
    dataString += "p" + String(flightPhase) + "\t";

    dataString += "A" + String(latitude, 6) + "\t";
    dataString += "O" + String(longitude, 6) + "\t";

    dataString += "h" + String(hora_value) + "\t";
    dataString += "n" + String(minuto_value) + "\t";
    dataString += "g" + String(precisao_value, 1) + "\t";

    dataString += "H" + String(altitude, 2) + "\t";
    dataString += "s" + String(sd_value) + "\t";

    dataString += "a" + String(apogeu_h, 2) + "\t";
    dataString += "t" + String(apogeu_t, 2) + "\t";

    dataString += "D" + String(pq_dn_altitude, 2) + "\t";
    dataString += "d" + String(pq_db_altitude, 2) + "\t";
    dataString += "M" + String(pq_mn_altitude, 2) + "\t";
    dataString += "m" + String(pq_mb_altitude, 2) + "\t";

    dataString += "c" + String(temp_value, 2) + "\t";

    dataString += "R" + String(euler_roll, 2) + "\t";
    dataString += "P" + String(euler_pitch, 2) + "\t";
    dataString += "Y" + String(euler_yaw, 2) + "\t";

    return dataString;
  }
  #endif
#endif

#if (LORA_MODE)
String buildLoraDataString(const SensorData &sensorData, const FlightEventsData &flightData) {
  String loraDataString = "";

  loraDataString += "T:" + String(sensorData.timestampMs / 1000.0f, 3) + '\t';

#if (GPS_MODE)
  if (sensorData.gpsAvailable) {
    loraDataString += "A" + String(sensorData.gpsLatitudeDeg, 6) + '\t';
    loraDataString += "O" + String(sensorData.gpsLongitudeDeg, 6);
  } else {
    loraDataString += "A~\tO~";
  }
#else
  loraDataString += "A~\tO~";
#endif

#if (BARO)
  loraDataString += "H" + String(sensorData.barometerFilteredAltitudeM, 2) + '\t';
  loraDataString += "ALTMAX" + String(flightData.maxTrackedAltitudeM, 2) + '\t';
#endif

#if (STORAGE_MODE && SD_CARD)
  loraDataString += "s" + String(sdWriteOk ? 1 : 0) + '\t';
#endif

#if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
  loraDataString += "a" + String(flightData.apogeeAltitudeM, 2) + '\t';
  loraDataString += "t" + String(flightData.apogeeTimeMs, 2) + '\t';
#endif

#if (PARACHUTE)
  #if (Parachute_FirstNormalEvent)
    loraDataString += "D" + String(flightData.parachuteFirstNormalAltitudeM, 2) + '\t';
  #endif

  #if (Parachute_FirstBackupEvent)
    loraDataString += "d" + String(flightData.parachuteFirstBackupAltitudeM, 2) + '\t';
  #endif

  #if (Parachute_SecondNormalEvent)
    loraDataString += "M" + String(flightData.parachuteSecondNormalAltitudeM, 2) + '\t';
  #endif

  #if (Parachute_SecondBackupEvent)
    loraDataString += "m" + String(flightData.parachuteSecondBackupAltitudeM, 2) + '\t';
  #endif
#endif

#if (BARO)
  loraDataString += "c" + String(sensorData.barometerTemperatureC, 2) + '\t';
#endif

#if (ENABLE_3D)
  if (sensorData.orientationAvailable) {
    loraDataString += "R" + String(sensorData.rollDeg, 2) + '\t';
    loraDataString += "P" + String(sensorData.pitchDeg, 2) + '\t';
    loraDataString += "Y" + String(sensorData.yawDeg, 2) + '\t';
  } else {
    loraDataString += "R~\tP~\tY~\t";
  }
#endif


  return loraDataString;
}
#endif

void printCommunicationHeadersIfNeeded() {
#if (NORMAL_SERIAL_PRINT_MODE || STORAGE_MODE)
  String dataHeader = buildDataHeader();
#endif

#if (LORA_MODE)
  String loraHeader = buildLoraHeader();
#endif

#if (NORMAL_SERIAL_PRINT_MODE)
  if (!serialHeaderPrinted) {
    Serial.println(dataHeader);
    serialHeaderPrinted = true;
  }
#endif

#if (LORA_MODE)
  if (!loraHeaderPrinted) {
    LoRa.println(loraHeader);
    loraHeaderPrinted = true;
  }
#endif

#if (STORAGE_MODE && SD_CARD)
  if (!sdHeaderPrinted) {
    logFile = SD.open(nameSD, FILE_APPEND);
    if (logFile) {
      logFile.println(dataHeader);
      logFile.close();
      sdHeaderPrinted = true;
    }
  }
#endif
}

#if (STORAGE_MODE)
#if (SD_CARD)
void createSdFileName() {
  int fileIndex = 0;
  unsigned long startTime = millis();
  String baseName = FILE_NAME;

  while (true) {
    if (millis() - startTime > 10000) {
      strcpy(nameSD, "error");
      break;
    }

    int numZeros = 8 - baseName.length() - String(fileIndex).length();
    String zeros = "";
    for (int i = 0; i < numZeros - 1; i++) {
      zeros += "0";
    }

    String fullName = "/" + baseName + zeros + String(fileIndex) + ".txt";
    fullName.toCharArray(nameSD, sizeof(nameSD));

    if (SD.exists(nameSD)) {
      fileIndex++;
    } else {
      File testFile = SD.open(nameSD, FILE_APPEND);
      if (testFile) {
        testFile.close();
      }
      break;
    }
  }
}
#endif
#endif
#endif
//--------------------------------------------------------- LORA MANAGER FUNCTIONS ---------------------------------------------------------
#if (LORA_MANAGER)

void verifyLoraCommand() {
  String command;
  if ((millis() - lastLoraCommandTime > LORA_COMMAND_TIMEOUT_MS) && changingFrequency) {
    LoRa.println(LORA_CHANGE_DONE_ERROR_FLIGHT);
    #if (LORA_RESET_TO_BASE_FREQ_ON_ERROR)
    loraManager.changeFrequency(LORA_BASE_CHAN, LORA_BASE_ADDH, LORA_BASE_ADDL);
    #endif
    changingFrequency = false;
  }

  if(loraManager.readLine(command)) {
    processLoraCommand(command);
  }
}

void processLoraCommand(const String &message) {
  if (message.startsWith("MUD4R_FR3Q_PFV.")) {
    String command = message.substring(String("MUD4R_FR3Q_PFV.").length());

    if (loraManager.decodeCommand(command, channel, addh, addl)) {
      String requestConfirmationMsg = LORA_CHANGE_REQUEST_CONFIRMATION_FLIGHT;

      String addhHex = String(addh, HEX);
      String addlHex = String(addl, HEX);

      addhHex.toUpperCase();
      addlHex.toUpperCase();

      if (addhHex.length() < 2) addhHex = "0" + addhHex;
      if (addlHex.length() < 2) addlHex = "0" + addlHex;

      requestConfirmationMsg += String(channel);
      requestConfirmationMsg += "_";
      requestConfirmationMsg += addhHex;
      requestConfirmationMsg += addlHex;
      requestConfirmationMsg += String("#");

      changingFrequency = true;
    } else {
      LoRa.println(LORA_CHANGE_DONE_ERROR_FLIGHT);
      changingFrequency = false;
    }
    lastLoraCommandTime = millis();
  }

  else if (message.startsWith("1SSO_MSM") && changingFrequency) {
    if (!loraManager.changeFrequency(channel, addh, addl)) {
      LoRa.println(LORA_CHANGE_DONE_ERROR_FLIGHT);
      changingFrequency = false;
    } 
  }

  else if (message.startsWith(LORA_CHANGE_FINAL_DONE_CONFIRMATION_GS_FLIGHT) && changingFrequency) {
    LoRa.println(LORA_CHANGE_FINAL_DONE_CONFIRMATION_FC_FLIGHT);
  }

  else if (message.startsWith(LORA_CHANGE_FINAL_DONE_COMMUNICATION_FC_FLIGHT) && changingFrequency) {
    changingFrequency = false;
  }
}

#endif
//--------------------------------------------------------- CALIBRATION FUNCTIONS ---------------------------------------------------------

//------------------------------ BAROMETER CALIBRATION ------------------------------//
#if (BARO)
void calibrateBarometer() {
  #if (AUTO_CALIBRATION_BARO)
    // Placeholder for future auto calibration
  #endif

  #if (USE_BMP180)
    for (int i = 0; i < BARO_STARTUP_FLUSH_READS; i++) {
      baro.lerTudo();
    }
    float pressureSum = 0.0f;
    for (int i = 0; i < NUM_STARTUP_READS; i++) {
      baro.lerTudo();
      pressureSum += baro.getPressao();
    }
    startupPressurePa = pressureSum / NUM_STARTUP_READS;
    startupAltitudeM = pressureToHeight(startupPressurePa);
  #endif

  #if (USE_BMP280)
    for (int i = 0; i < BARO_STARTUP_FLUSH_READS; i++) {
      baro.readAll();
    }

    float pressureSum = 0.0f;
    int validSamples = 0;

    for (int i = 0; i < NUM_STARTUP_READS; i++) {
      baro.readAll();
      if (baro) {
        pressureSum += baro.getPressure();
        validSamples++;
      }
    }

    if (validSamples > 0) {
      startupPressurePa = pressureSum / validSamples;
      startupAltitudeM = pressureToHeight(startupPressurePa);
    } else {
      startupPressurePa = NAN;
      startupAltitudeM = NAN;
    }
  #endif

  #if (USE_BME280)
    baro.setSettings(Bme280Settings::indoor());

    for (int i = 0; i < BARO_STARTUP_FLUSH_READS; i++) {
      baro.getPressure();
    }

    float pressureSum = 0.0f;
    int validSamples = 0;

    for (int i = 0; i < NUM_STARTUP_READS; i++) {
      float pressure = baro.getPressure();
      if (pressure != 0.0f && !isnan(pressure)) {
        pressureSum += pressure;
        validSamples++;
        #if (DEBUG_PRINTS)
        Serial.print(F("Startup Read ")); Serial.print(i + 1); Serial.print(F(": Pressure = ")); Serial.print(pressure); Serial.println(F(" Pa"));
        #endif
      }
    }

    if (validSamples > 0) {
      startupPressurePa = pressureSum / validSamples;
      startupAltitudeM = pressureToHeight(startupPressurePa);
    } else {
      startupPressurePa = NAN;
      startupAltitudeM = NAN;
    }
  #endif

  #if (USE_BMP380)
    // Placeholder for future implementation
    startupPressurePa = 0.0f;
    startupAltitudeM = pressureToHeight(startupPressurePa);
  #endif

  #if (ESSENTIALS_ONLY_SERIAL_PRINT_MODE || NORMAL_SERIAL_PRINT_MODE)
    Serial.print(F("Startup Pressure (Pa): "));
    Serial.println(startupPressurePa);
    Serial.print(F("Startup Altitude (m): "));
    Serial.println(startupAltitudeM);
  #endif
}
#endif

//------------------------------ SIMULATION CALIBRATION ------------------------------//
#if (URD_APP_SIMULATION_MODE)
void calibrateSimulation() {
  startupPressurePa = urdApp.getPressure();
  startupAltitudeM = pressureToHeight(startupPressurePa);
}
#endif

//------------------------------ BAROMETER/SIMULATION RESTART PROTECTION CALIBRATION ------------------------------//
#if ((BARO || URD_APP_SIMULATION_MODE) && RESTART_PROTECTION)
void applyBarometerRestartProtection() {
  bool validPressure = true;

  if ((startupPressurePa >= BARO_RESTART_MAX_PRESSURE_PA) || (startupPressurePa <= BARO_RESTART_MIN_PRESSURE_PA)) {
    validPressure = false;
  }

  if (!validPressure && flightActive) {
    #if defined(PLACA_ESSENTIALS_V1) || defined(PLACA_MAIN_PTH) || defined(PLACA_MAIN_SMD) || defined(PLACA_MAIN_IC)
      startupPressurePa = prefs.getFloat("startupPressurePa", 0.0f);
      startupAltitudeM = pressureToHeight(startupPressurePa);
    #endif
    #if (ESSENTIALS_ONLY_SERIAL_PRINT_MODE || NORMAL_SERIAL_PRINT_MODE)
    Serial.println(F("RESTART PROTECTION"));
    #endif
  } else {
    flightActive = false;
    #if defined(PLACA_ESSENTIALS_V1) || defined(PLACA_MAIN_PTH) || defined(PLACA_MAIN_SMD) || defined(PLACA_MAIN_IC)
      prefs.putFloat("startupPressurePa", startupPressurePa);
      prefs.putBool("flightActive", flightActive);
    #endif
    #if (ESSENTIALS_ONLY_SERIAL_PRINT_MODE || NORMAL_SERIAL_PRINT_MODE)
    Serial.println(F("NO RESTART PROTECTION"));
    #endif
  }
}
#endif

//------------------------------ ACCELEROMETER CALIBRATION ------------------------------//
#if (ACCEL)
void applyAccelerometerCalibration(float &x, float &y, float &z) {
  #if (USE_ICM20948_ACCEL)
    #if (AUTO_CALIBRATION_ACCEL)
      // already corrected internally by the library
    #else
      #if (ACCEL_FULL_CALIBRATION_MODE)
        // already corrected internally by imu.setAccOffsets(min/max)
      #else
        // already corrected internally by imu.setAccOffsets(min/max)
      #endif
    #endif
  #else
    #if (AUTO_CALIBRATION_ACCEL)
      // placeholder for future non-ICM automatic calibration
    #else
      x = (x - ACCEL_OFFSET_X) * ACCEL_SCALE_X;
      y = (y - ACCEL_OFFSET_Y) * ACCEL_SCALE_Y;
      z = (z - ACCEL_OFFSET_Z) * ACCEL_SCALE_Z;
    #endif
  #endif
}
#endif

//------------------------------ GYROSCOPE CALIBRATION ------------------------------//
#if (GYRO)
void applyGyroscopeCalibration(float &x, float &y, float &z) {
  #if (USE_ICM20948_GYRO)
    #if (AUTO_CALIBRATION_GYRO)
      // already corrected internally by the library
    #else
      // already corrected internally by imu.setGyrOffsets()
    #endif
  #else
    #if (AUTO_CALIBRATION_GYRO)
      // placeholder for future non-ICM automatic calibration
    #else
      x = (x - GYRO_OFFSET_X) * GYRO_SCALE_X;
      y = (y - GYRO_OFFSET_Y) * GYRO_SCALE_Y;
      z = (z - GYRO_OFFSET_Z) * GYRO_SCALE_Z;
    #endif
  #endif
}
#endif

//------------------------------ MAGNETOMETER CALIBRATION ------------------------------//
#if (MAG)
void applyMagnetometerCalibration(float &x, float &y, float &z) {
  #if (USE_ICM20948_MAG)
    // Placeholder for future ICM20948 magnetometer calibration 
  #else
    x = (x - MAG_BIAS_X) * MAG_SCALE_X;
    y = (y - MAG_BIAS_Y) * MAG_SCALE_Y;
    z = (z - MAG_BIAS_Z) * MAG_SCALE_Z;
  #endif
}
#endif

//------------------------------ HUMIDITY CALIBRATION ------------------------------//
#if (HUMIDITY)
void calibrateHumiditySensor() {
  // Placeholder
}
#endif

//------------------------------ ICM20948 LIBRARY CALIBRATION ------------------------------//
#if (USE_ICM20948_ACCEL) || (USE_ICM20948_GYRO) || (USE_ICM20948_MAG)
void initCalibrationIcm20948() {
  #if (AUTO_CALIBRATION_ACCEL) || (AUTO_CALIBRATION_GYRO)
    imu.autoOffsets();
  #else
    #if !(AUTO_CALIBRATION_ACCEL)
      #if (ACCEL_FULL_CALIBRATION_MODE)
        imu.setAccOffsets(
          ACCEL_MIN_X, ACCEL_MAX_X,
          ACCEL_MIN_Y, ACCEL_MAX_Y,
          ACCEL_MIN_Z, ACCEL_MAX_Z
        );
      #else
        imu.setAccOffsets(ACCEL_OFFSET_X, ACCEL_OFFSET_Y, ACCEL_OFFSET_Z);
      #endif
    #endif

    #if !(AUTO_CALIBRATION_GYRO)
      imu.setGyrOffsets(GYRO_OFFSET_X, GYRO_OFFSET_Y, GYRO_OFFSET_Z);
    #endif
  #endif
}
#endif


//--------------------------------------------------------- INITIALIZATION FUNCTIONS ---------------------------------------------------------
//------------------------------ FLIGHT EVENTS INITIALIZATION ------------------------------//
void initFlightEventsData() {
  if (gFlightEventsMutex == nullptr) {
    gFlightEventsMutex = xSemaphoreCreateMutex();
  }

  FlightEventsData data = {};

  data.timestampMs = millis();
  data.inputAltitudeM = 0.0f;

#if (WUF_PROTECTION)
  data.wufLockActive = true;
#endif

#if (TRANSONIC_PROTECTION)
  data.transonicLockActive = true;
#endif

#if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
  data.descending = false;
  data.apogeeDetected = false;
  data.apogeeAltitudeM = 0.0f;
  data.apogeeTimeMs = 0;
  data.maxTrackedAltitudeM = 0.0f;
#endif

#if (PARACHUTE)
  #if (Parachute_FirstNormalEvent)
    data.parachuteFirstNormalData = 0;
    data.parachuteFirstNormalContinuity = false;
    data.parachuteFirstNormalAltitudeM = 0.0f;
    data.parachuteFirstNormalTimeMs = 0;
  #endif

  #if (Parachute_FirstBackupEvent)
    data.parachuteFirstBackupData = 0;
    data.parachuteFirstBackupContinuity = false;
    data.parachuteFirstBackupAltitudeM = 0.0f;
    data.parachuteFirstBackupTimeMs = 0;
  #endif

  #if (Parachute_SecondNormalEvent)
    data.parachuteSecondNormalData = 0;
    data.parachuteSecondNormalContinuity = false;
    data.parachuteSecondNormalAltitudeM = 0.0f;
    data.parachuteSecondNormalTimeMs = 0;
  #endif

  #if (Parachute_SecondBackupEvent)
    data.parachuteSecondBackupData = 0;
    data.parachuteSecondBackupContinuity = false;
    data.parachuteSecondBackupAltitudeM = 0.0f;
    data.parachuteSecondBackupTimeMs = 0;
  #endif
#endif

  updateFlightEventsData(data);
}

//------------------------------ FLIGHT STATE INITIALIZATION ------------------------------//
void initFlightState() {
  if (gFlightStateMutex == nullptr) {
    gFlightStateMutex = xSemaphoreCreateMutex();
  }

  FlightState state = {};

  state.phase = FLIGHT_PHASE_INITIALIZATION;
  state.wufPassed = false;
  state.boostDetected = false;
  state.firstEventTriggered = false;
  state.secondEventTriggered = false;
  state.landedDetected = false;

  state.currentAltitudeM = 0.0f;
  state.maxAltitudeM = 0.0f;
  state.resultantAccelerationMs2 = 0.0f;

  state.phaseTimestampMs = millis();
  state.lastLandingCheckMs = millis();
  state.landingReferenceAltitudeM = 0.0f;

  updateFlightState(state);
}

//------------------------------ SYSTEM STATES INITIALIZATION ------------------------------//
void initSystemStates() {
  if (gSystemStatesMutex == nullptr) {
    gSystemStatesMutex = xSemaphoreCreateMutex();
  }

  SystemStates states = {};

  states.startupDone = false;

#if (BARO)
  states.barometerInitOk = false;
  states.barometerRuntimeOk = false;
#endif

#if (URD_APP_SIMULATION_MODE)
  states.simulationInitOk = false;
  states.simulationRuntimeOk = false;
#endif

#if (COMMUNICATION_ON)
  states.storageInitOk = false;
  states.storageRuntimeOk = false;
#endif

#if (ACCEL)
  states.accelerometerInitOk = false;
  states.accelerometerRuntimeOk = false;
#endif

#if (GYRO)
  states.gyroscopeInitOk = false;
  states.gyroscopeRuntimeOk = false;
#endif

#if (MAG)
  states.magnetometerInitOk = false;
  states.magnetometerRuntimeOk = false;
#endif

#if (HUMIDITY)
  states.humidityInitOk = false;
  states.humidityRuntimeOk = false;
#endif

#if (GPS_MODE)
  states.gpsInitOk = false;
  states.gpsRuntimeOk = false;
#endif

#if (BATTERY_MONITOR)
  states.batteryInitOk = false;
  states.batteryRuntimeOk = false;
#endif

#if (ENABLE_3D)
  states.orientationInitOk = false;
  states.orientationRuntimeOk = false;
#endif

  updateSystemStates(states);
}

//------------------------------ SENSOR DATA INITIALIZATION ------------------------------//
#if (SENSORS_ON)
void initSensorData() {
  if (gSensorMutex == nullptr) {
    gSensorMutex = xSemaphoreCreateMutex();
  }

  SensorData data = {};

  data.timestampMs = millis();
  data.frequencyHz = 0;

  updateSensorData(data);
}
#endif

//------------------------------ BAROMETER INITIALIZATION ------------------------------//
#if (BARO)
bool initBarometer() {
  
  #if (USE_BMP180)
    if (!baro.begin()) {
      Serial.println(F("BMP180 err"));
      return false;
    }
    Serial.println(F("BMP180 ok"));
  #endif

  #if (USE_BMP280)
    baro.begin();
    if (baro) {
      Serial.println(F("BMP280 ok"));
    } else {
      Serial.println(F("BMP280 err"));
      return false;
    }
  #endif

  #if (USE_BME280)
    if (!baro.begin(Bme280TwoWireAddress::Primary)) {
      Serial.println(F("BME280 err"));
      return false;
    }
    Serial.println(F("BME280 ok"));
  #endif

  #if (USE_BMP380)
    Serial.println(F("BMP380 placeholder"));
  #endif

  calibrateBarometer();

  #if (RESTART_PROTECTION)
    applyBarometerRestartProtection();
  #endif

  return true;
}
#endif

//------------------------------ SIMULATION INITIALIZATION ------------------------------//
bool initSimulation() {
  Serial.println(F("Simulation mode ok"));
  return true;
}

//------------------------------ ACCELEROMETER INITIALIZATION ------------------------------//
#if (ACCEL)
bool initAccelerometer() {
  #if (USE_MPU9250_ACCEL)
    accel.begin();
    if (accel) {
      Serial.println(F("Accel ok"));
    } else {
      Serial.println(F("Accel err"));
      return false;
    }
  #endif

  #if (USE_ICM20948_ACCEL)
    if (!imu.init()) {
      Serial.println(F("ICM20948 accel err"));
      Serial.println(F("ICM20948 gyro err"));
      return false;
    }
    Serial.println(F("ICM20948 accel ok"));
    Serial.println(F("ICM20948 gyro ok"));

    imu.setAccRange(ICM20948_ACC_RANGE_2G);
    imu.setAccDLPF(ICM20948_DLPF_6);

    imu.setGyrRange(ICM20948_GYRO_RANGE_250);
    imu.setGyrDLPF(ICM20948_DLPF_6);
  #endif



  #if (USE_ICM20948_ACCEL)
    initCalibrationIcm20948();
  #endif

  return true;
}
#endif

//------------------------------ GYROSCOPE INITIALIZATION ------------------------------//
#if (GYRO)
bool initGyroscope() {
  #if (USE_MPU9250_GYRO)
    gyro.begin();
    if (gyro) {
      Serial.println(F("Gyro ok"));
    } else {
      Serial.println(F("Gyro err"));
      return false;
    }
  #endif

  #if (USE_ICM20948_GYRO)
  #if (!ACCEL)
    if (!imu.init()) {
      Serial.println(F("ICM20948 gyro err"));
      return false;
    }
    Serial.println(F("ICM20948 gyro ok"));

    imu.setGyrRange(ICM20948_GYRO_RANGE_250);
    imu.setGyrDLPF(ICM20948_DLPF_6);
  #endif
  #endif

  

  #if (!USE_ICM20948_ACCEL)
    initCalibrationIcm20948();
  #endif
  return true;
}
#endif

//------------------------------ MAGNETOMETER INITIALIZATION ------------------------------//
#if (MAG)
bool initMagnetometer() {
  #if (USE_AK8963_MAG)
    mag.begin();
    if (mag) {
      Serial.println(F("Mag ok"));
    } else {
      Serial.println(F("Mag err"));
      return false;
    }
  #endif

  #if (USE_ICM20948_MAG)
    if (!imu.initMagnetometer()) {
      Serial.println(F("ICM20948 mag err"));
      return false;
    }
    Serial.println(F("ICM20948 mag ok"));

    imu.setMagOpMode(AK09916_CONT_MODE_20HZ);
  #endif

  
  
  #if (!USE_ICM20948_ACCEL && !USE_ICM20948_GYRO)
    initCalibrationIcm20948();
  #endif
  
  return true;
}
#endif

//------------------------------ HUMIDITY INITIALIZATION ------------------------------//
#if (HUMIDITY)
bool initHumiditySensor() {
  humiditySensor.begin();
  Serial.println(F("DHT22 initialized"));
  calibrateHumiditySensor();
  return true;
}
#endif

//------------------------------ GPS INITIALIZATION ------------------------------//
#if (GPS_MODE)
bool initGps() {
  #if defined(PLACA_ESSENTIALS_V1) || defined(PLACA_MAIN_PTH) || defined(PLACA_MAIN_SMD) || defined(PLACA_MAIN_IC)
    GPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_TX, GPS_RX);
  #else
    GPS.begin(GPS_BAUD_RATE);
  #endif

  #if (ESSENTIALS_ONLY_SERIAL_PRINT_MODE || NORMAL_SERIAL_PRINT_MODE)
    Serial.println(F("GPS initialized"));
  #endif
  return true;
}
#endif

//------------------------------ BATTERY MONITOR INITIALIZATION ------------------------------//
#if (BATTERY_MONITOR)
bool initBatteryMonitor() {
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  #if (ESSENTIALS_ONLY_SERIAL_PRINT_MODE || NORMAL_SERIAL_PRINT_MODE)
    Serial.println(F("Battery monitor initialized"));
  #endif
  return true;
}
#endif



//------------------------------ 3D ORIENTATION INITIALIZATION ------------------------------//
#if (ENABLE_3D)
bool initOrientation() {
  orientation.begin(SENSOR_TASK_FREQUENCY_HZ);

  orientationStartTimeMs = millis();
  orientationWarmupDone = false;
  orientationOffsetsReady = false;

  orientationOffsetSampleCount = 0;

  rollOffsetSumDeg = 0.0f;
  pitchOffsetSumDeg = 0.0f;
  yawOffsetSumDeg = 0.0f;

  rollOffsetDeg = 0.0f;
  pitchOffsetDeg = 0.0f;
  yawOffsetDeg = 0.0f;

  Serial.println(F("3D ok"));
  return true;
}
#endif

//------------------------------ ALL SYSTEM INITIALIZATION ------------------------------//
void initAll() {
  SystemStates states = {};

#if (SENSORS_ON)
  setupSensorsModule();
#endif

#if (BARO)
  states.barometerInitOk = initBarometer();
#endif

#if (ACCEL)
  states.accelerometerInitOk = initAccelerometer();
#endif

#if (GYRO)
  states.gyroscopeInitOk = initGyroscope();
#endif

#if (MAG)
  states.magnetometerInitOk = initMagnetometer();
#endif

#if (HUMIDITY)
  states.humidityInitOk = initHumiditySensor();
#endif

#if (GPS_MODE)
  states.gpsInitOk = initGps();
#endif

#if (COMMUNICATION_ON)
  states.storageInitOk = initCommunication();
#endif

#if (BATTERY_MONITOR)
  states.batteryInitOk = initBatteryMonitor();
#endif

#if (ENABLE_3D)
  states.orientationInitOk = initOrientation();
#endif

  states.startupDone = true;

  updateSystemStates(states);
}

//------------------------------ COMMUNICATION INITIALIZATION ------------------------------//
bool initCommunication() {
  bool state = true;
#if (LORA_MODE)
  #if defined(PLACA_ESSENTIALS_V1) || defined(PLACA_MAIN_PTH) || defined(PLACA_MAIN_SMD) || defined(PLACA_MAIN_IC)
    LoRa.begin(LORA_BAUD_RATE, SERIAL_8N1, LORA_TX, LORA_RX);
  #else
    LoRa.begin(LORA_BAUD_RATE);
  #endif

  pinMode(M0_LORA_PIN, OUTPUT);
  pinMode(M1_LORA_PIN, OUTPUT);
  pinMode(AUX_LORA_PIN, INPUT);

  digitalWrite(M0_LORA_PIN, LOW);
  digitalWrite(M1_LORA_PIN, LOW);

  #if (LORA_MANAGER)
      LoRa.setTimeout(500);
      loraManager.setPrint(true);
      loraManager.setDebug(false);
      loraManager.initE32(e32Module);
  #endif

  #if (ESSENTIALS_ONLY_SERIAL_PRINT_MODE || NORMAL_SERIAL_PRINT_MODE)
    Serial.println(F("LoRa initialized"));
  #endif
#endif

#if (STORAGE_MODE && SD_CARD)
  sdInitOk = false;

  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    SPI_SD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    sdInitOk = SD.begin(SD_CS, SPI_SD, 40000000);
  #elif defined(ARDUINO_ARCH_ESP32)
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    sdInitOk = SD.begin(SD_CS, SPI);
  #else
    sdInitOk = SD.begin(SD_CS);
  #endif

  if (sdInitOk) {
    createSdFileName();
    #if (ESSENTIALS_ONLY_SERIAL_PRINT_MODE || NORMAL_SERIAL_PRINT_MODE)
      Serial.println(F("SD ok"));
      Serial.println(nameSD);
    #endif
  } else {
    #if (ESSENTIALS_ONLY_SERIAL_PRINT_MODE || NORMAL_SERIAL_PRINT_MODE)
      Serial.println(F("SD err"));
    #endif
  }
  state = sdInitOk;
#endif
  return state;
}

//------------------------------ FLIGHT EVENTS INITIALIZATION ------------------------------//
void initFlightEvents() {
#if (PARACHUTE)
  #if (Parachute_FirstNormalEvent)
    parachuteFirstNormal.begin();
  #endif

  #if (Parachute_FirstBackupEvent)
    parachuteFirstBackup.begin();
  #endif

  #if (Parachute_SecondNormalEvent)
    parachuteSecondNormal.begin();
  #endif

  #if (Parachute_SecondBackupEvent)
    parachuteSecondBackup.begin();
  #endif
#endif
}

//--------------------------------------------------------- SENSOR READ FUNCTIONS ---------------------------------------------------------

//------------------------------ BAROMETER READ ------------------------------//
#if (BARO)
void readBarometer(SensorData &data) {
  #if (USE_BMP180)
    bool readOk = baro.lerTudo();
    data.barometerAvailable = readOk;

    if (readOk) {
      data.barometerTemperatureC = baro.getTemperatura();
      data.barometerPressurePa   = baro.getPressao();
      data.barometerAltitudeM    = baro.getAltitude() - startupAltitudeM;
    }
  #endif

  #if (USE_BMP280)
    if (baro) {
      data.barometerAvailable    = true;
      data.barometerTemperatureC = baro.getTemperature();
      data.barometerPressurePa   = baro.getPressure();
      data.barometerAltitudeM    = pressureToHeight(data.barometerPressurePa) - startupAltitudeM;
    } else {
      data.barometerAvailable = false;
    }
  #endif

  #if (USE_BME280)
      float pressure = baro.getPressure();
      float temperature = baro.getTemperature();
      float humidity = baro.getHumidity();
      if (pressure != 0.0f && !isnan(pressure)) {
        data.barometerAvailable    = true;
        data.barometerPressurePa   = pressure;
        data.barometerTemperatureC = temperature;
        data.humidityPercent = humidity;
        data.barometerAltitudeM    = pressureToHeight(data.barometerPressurePa) - startupAltitudeM;
      } else {
          data.barometerAvailable = false;
      }
  #endif

  #if (USE_BMP380)
    data.barometerAvailable = false;
  #endif

  #if (BARO_FILTERS)
    #if (KALMAN_MovingAverage_BARO)
      data.barometerFilteredAltitudeM = kalmanBaroAltitude.updateEstimate(data.barometerAltitudeM);
    #else
      baroFilter1.aplicarFiltro(data.barometerAltitudeM);
      baroFilter2.aplicarFiltro(baroFilter1.getMedia());
      baroFilter3.aplicarFiltro(baroFilter2.getMedia());
      data.barometerFilteredAltitudeM = baroFilter3.getMedia();
    #endif
  #else
    data.barometerFilteredAltitudeM = data.barometerAltitudeM;
  #endif

}
#endif

//------------------------------ SIMULATION READ ------------------------------//

#if (URD_APP_SIMULATION_MODE)
void readSimulation(SensorData &data) {
  data.simulationAvailable =   urdApp.updateSimulation();
  data.simulationPressurePa = urdApp.getPressure();
  data.simulationAltitudeM = pressureToHeight(data.simulationPressurePa) - startupAltitudeM;
}
#endif

//------------------------------ ACCELEROMETER READ ------------------------------//
#if (ACCEL)
void readAccelerometer(SensorData &data) {
  #if (USE_MPU9250_ACCEL)
    if (accel) {
      data.accelerometerAvailable = true;

      data.accelX = accel.getX();
      data.accelY = accel.getY();
      data.accelZ = accel.getZ();


      #if (ACCEL_FILTERS)
        #if (KALMAN_MovingAverage_ACCEL)
          data.accelFilteredX = kalmanAccelX.updateEstimate(data.accelX);
          data.accelFilteredY = kalmanAccelY.updateEstimate(data.accelY);
          data.accelFilteredZ = kalmanAccelZ.updateEstimate(data.accelZ);
        #else
          accelFilterX.aplicarFiltro(data.accelX);
          accelFilterY.aplicarFiltro(data.accelY);
          accelFilterZ.aplicarFiltro(data.accelZ);

          data.accelFilteredX = accelFilterX.getMedia();
          data.accelFilteredY = accelFilterY.getMedia();
          data.accelFilteredZ = accelFilterZ.getMedia();
        #endif
      #else
        data.accelFilteredX = data.accelX;
        data.accelFilteredY = data.accelY;
        data.accelFilteredZ = data.accelZ;
      #endif
    } else {
      data.accelerometerAvailable = false;
    }
  #endif

  #if (USE_ICM20948_ACCEL)
    xyzFloat acc;
    imu.readSensor();
    imu.getGValues(&acc);
    data.accelerometerAvailable = true;

    data.accelX = acc.x * 9.81;
    data.accelY = acc.y * 9.81;
    data.accelZ = acc.z * 9.81;

    applyAccelerometerCalibration(data.accelX, data.accelY, data.accelZ);

    #if (ACCEL_FILTERS)
      #if (KALMAN_MovingAverage_ACCEL)
        data.accelFilteredX = kalmanAccelX.updateEstimate(data.accelX);
        data.accelFilteredY = kalmanAccelY.updateEstimate(data.accelY);
        data.accelFilteredZ = kalmanAccelZ.updateEstimate(data.accelZ);
      #else
        accelFilterX.aplicarFiltro(data.accelX);
        accelFilterY.aplicarFiltro(data.accelY);
        accelFilterZ.aplicarFiltro(data.accelZ);

        data.accelFilteredX = accelFilterX.getMedia();
        data.accelFilteredY = accelFilterY.getMedia();
        data.accelFilteredZ = accelFilterZ.getMedia();
      #endif
    #else
      data.accelFilteredX = data.accelX;
      data.accelFilteredY = data.accelY;
      data.accelFilteredZ = data.accelZ;
    #endif
  #endif
}
#endif

//------------------------------ GYROSCOPE READ ------------------------------//
#if (GYRO)
void readGyroscope(SensorData &data) {
  #if (USE_MPU9250_GYRO)
    if (gyro) {
      data.gyroscopeAvailable = true;

      data.gyroX = gyro.getX();
      data.gyroY = gyro.getY();
      data.gyroZ = gyro.getZ();


      #if (GYRO_FILTERS)
        #if (KALMAN_MovingAverage_GYRO)
          data.gyroFilteredX = kalmanGyroX.updateEstimate(data.gyroX);
          data.gyroFilteredY = kalmanGyroY.updateEstimate(data.gyroY);
          data.gyroFilteredZ = kalmanGyroZ.updateEstimate(data.gyroZ);
        #else
          gyroFilterX.aplicarFiltro(data.gyroX);
          gyroFilterY.aplicarFiltro(data.gyroY);
          gyroFilterZ.aplicarFiltro(data.gyroZ);

          data.gyroFilteredX = gyroFilterX.getMedia();
          data.gyroFilteredY = gyroFilterY.getMedia();
          data.gyroFilteredZ = gyroFilterZ.getMedia();
        #endif
      #else
        data.gyroFilteredX = data.gyroX;
        data.gyroFilteredY = data.gyroY;
        data.gyroFilteredZ = data.gyroZ;
      #endif
    } else {
      data.gyroscopeAvailable = false;
    }
  #endif

  #if (USE_ICM20948_GYRO)
    xyzFloat gyr;
    #if (!ACCEL)
    imu.readSensor();
    #endif
    imu.getGyrValues(&gyr);
    data.gyroscopeAvailable = true;

    data.gyroX = gyr.x;
    data.gyroY = gyr.y;
    data.gyroZ = gyr.z;

    applyGyroscopeCalibration(data.gyroX, data.gyroY, data.gyroZ);
    #if (GYRO_FILTERS)
      #if (KALMAN_MovingAverage_GYRO)
        data.gyroFilteredX = kalmanGyroX.updateEstimate(data.gyroX);
        data.gyroFilteredY = kalmanGyroY.updateEstimate(data.gyroY);
        data.gyroFilteredZ = kalmanGyroZ.updateEstimate(data.gyroZ);
      #else
        gyroFilterX.aplicarFiltro(data.gyroX);
        gyroFilterY.aplicarFiltro(data.gyroY);
        gyroFilterZ.aplicarFiltro(data.gyroZ);

        data.gyroFilteredX = gyroFilterX.getMedia();
        data.gyroFilteredY = gyroFilterY.getMedia();
        data.gyroFilteredZ = gyroFilterZ.getMedia();
      #endif
    #else
      data.gyroFilteredX = data.gyroX;
      data.gyroFilteredY = data.gyroY;
      data.gyroFilteredZ = data.gyroZ;
    #endif
  #endif
}
#endif

//------------------------------ MAGNETOMETER READ ------------------------------//
#if (MAG)
void readMagnetometer(SensorData &data) {
  #if (USE_AK8963_MAG)
    if (mag) {
      data.magnetometerAvailable = true;

      data.magX = mag.getX();
      data.magY = mag.getY();
      data.magZ = mag.getZ();

      #if (MAG_FILTERS)
        #if (KALMAN_MovingAverage_MAG)
          data.magFilteredX = kalmanMagX.updateEstimate(data.magX);
          data.magFilteredY = kalmanMagY.updateEstimate(data.magY);
          data.magFilteredZ = kalmanMagZ.updateEstimate(data.magZ);
        #else
          magFilterX.aplicarFiltro(data.magX);
          magFilterY.aplicarFiltro(data.magY);
          magFilterZ.aplicarFiltro(data.magZ);

          data.magFilteredX = magFilterX.getMedia();
          data.magFilteredY = magFilterY.getMedia();
          data.magFilteredZ = magFilterZ.getMedia();
        #endif
      #else
        data.magFilteredX = data.magX;
        data.magFilteredY = data.magY;
        data.magFilteredZ = data.magZ;
      #endif
    } else {
      data.magnetometerAvailable = false;
    }
  #endif

  #if (USE_ICM20948_MAG)
    xyzFloat magValues;
    imu.getMagValues(&magValues);
    data.magnetometerAvailable = true;

    data.magX = magValues.x;
    data.magY = -1 * magValues.y; // Inverting Y axis to match accel and gyro orientation
    data.magZ = -1 * magValues.z; // Inverting Z axis to match accel and gyro orientation

    applyMagnetometerCalibration(data.magX, data.magY, data.magZ);
    #if (MAG_FILTERS)
      #if (KALMAN_MovingAverage_MAG)
        data.magFilteredX = kalmanMagX.updateEstimate(data.magX);
        data.magFilteredY = kalmanMagY.updateEstimate(data.magY);
        data.magFilteredZ = kalmanMagZ.updateEstimate(data.magZ);
      #else
        magFilterX.aplicarFiltro(data.magX);
        magFilterY.aplicarFiltro(data.magY);
        magFilterZ.aplicarFiltro(data.magZ);

        data.magFilteredX = magFilterX.getMedia();
        data.magFilteredY = magFilterY.getMedia();
        data.magFilteredZ = magFilterZ.getMedia();
      #endif
    #else
      data.magFilteredX = data.magX;
      data.magFilteredY = data.magY;
      data.magFilteredZ = data.magZ;
    #endif
  #endif
}
#endif

//------------------------------ HUMIDITY READ ------------------------------//
#if (HUMIDITY)
void readHumiditySensor(SensorData &data) {
  data.humidityTemperatureC = humiditySensor.readTemperature();
  data.humidityRelativePercent = humiditySensor.readHumidity();

  if (!isnan(data.humidityTemperatureC) && !isnan(data.humidityRelativePercent)) {
    data.humidityAvailable = true;
  } else {
    data.humidityAvailable = false;
  }
}
#endif

//------------------------------ GPS READ ------------------------------//
#if (GPS_MODE)
void readGps(SensorData &data) {
  while (GPS.available()) {
    gps.encode(GPS.read());
  }

  data.gpsAvailable = gps.location.isValid();

  if (gps.location.isValid()) {
    data.gpsLatitudeDeg = gps.location.lat();
    data.gpsLongitudeDeg = gps.location.lng();
  } else {
    data.gpsLatitudeDeg = NAN;
    data.gpsLongitudeDeg = NAN;
  }

  if (gps.altitude.isValid()) {
    data.gpsAltitudeM = gps.altitude.meters();
  } else {
    data.gpsAltitudeM = NAN;
  }

  if (gps.speed.isValid()) {
    data.gpsSpeedKmph = gps.speed.kmph();
  } else {
    data.gpsSpeedKmph = NAN;
  }

  if (gps.satellites.isValid()) {
    data.gpsSatellites = gps.satellites.value();
  } else {
    data.gpsSatellites = 0;
  }

  if (gps.hdop.isValid()) {
    data.gpsHdop = gps.hdop.hdop();
  } else {
    data.gpsHdop = NAN;
  }
}
#endif

//------------------------------ BATTERY READ ------------------------------//
#if (BATTERY_MONITOR)
void readBatteryMonitor(SensorData &data) {
  int raw = analogRead(BATTERY_PIN);

  float adcVoltage = (raw / 4095.0f) * ADC_REFERENCE_VOLTAGE;
  float batteryVoltage = adcVoltage * BATTERY_DIVIDER_RATIO;

  data.batteryState = true;
  data.batteryVoltage = batteryVoltage; 

  float percent = (batteryVoltage - BATTERY_EMPTY_VOLTAGE) /
                  (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE) * 100.0f;

  if (percent < 0.0f) percent = 0.0f;
  if (percent > 100.0f) percent = 100.0f;

  data.batteryPercent = percent;
}
#endif

//------------------------------ FLIGHT STATE UPDATE ------------------------------//
void updateFlightStateFromSensors(const SensorData &sensorData,
                                  const FlightEventsData &flightEvents,
                                  FlightState &flightState) {
  float currentAltitudeM = 0.0f;

#if (URD_APP_SIMULATION_MODE)
  currentAltitudeM = sensorData.simulationAltitudeM;
#elif (BARO)
  currentAltitudeM = sensorData.barometerFilteredAltitudeM;
#endif

  flightState.currentAltitudeM = currentAltitudeM;

#if (ACCEL)
  flightState.resultantAccelerationMs2 =
    sqrtf(sensorData.accelFilteredX * sensorData.accelFilteredX +
          sensorData.accelFilteredY * sensorData.accelFilteredY +
          sensorData.accelFilteredZ * sensorData.accelFilteredZ);
#else
  flightState.resultantAccelerationMs2 = 0.0f;
#endif

#if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
  flightState.maxAltitudeM = flightEvents.maxTrackedAltitudeM;
#endif

  // INITIALIZATION -> PRE_FLIGHT
  if (flightState.phase == FLIGHT_PHASE_INITIALIZATION) {
    SystemStates states = {};
    copySystemStates(states);

    if (states.startupDone) {
      flightState.phase = FLIGHT_PHASE_PRE_FLIGHT;
      flightState.phaseTimestampMs = sensorData.timestampMs;
    }
  }

  // PRE_FLIGHT -> BOOST
  else if (flightState.phase == FLIGHT_PHASE_PRE_FLIGHT) {

#if (WUF_PROTECTION)
    if (!flightEvents.wufLockActive) {
      flightState.phase = FLIGHT_PHASE_BOOST;
      flightState.wufPassed = true;
      flightState.boostDetected = true;
      flightState.phaseTimestampMs = sensorData.timestampMs;
    }
#else
    // Sem WUF, entra em BOOST assim que sair do pre-flight.
    flightState.phase = FLIGHT_PHASE_BOOST;
    flightState.wufPassed = true;
    flightState.boostDetected = true;
    flightState.phaseTimestampMs = sensorData.timestampMs;
#endif

  }
  // COAST is missing 
  // BOOST -> FIRST_EVENT
  else if (flightState.phase == FLIGHT_PHASE_BOOST) {

#if (PARACHUTE)
    bool firstPqdTriggered = false;

  #if (Parachute_FirstNormalEvent)
    if (parachuteFirstNormal.getPqdHeight() != 0.0f) {
      firstPqdTriggered = true;
    }
  #endif

  #if (Parachute_FirstBackupEvent)
    if (parachuteFirstBackup.getPqdHeight() != 0.0f) {
      firstPqdTriggered = true;
    }
  #endif

    if (firstPqdTriggered) {
      flightState.phase = FLIGHT_PHASE_FIRST_EVENT;
      flightState.firstEventTriggered = true;
      flightState.phaseTimestampMs = sensorData.timestampMs;
    }
#endif

  }

  // FIRST_EVENT -> SECOND_EVENT
  else if (flightState.phase == FLIGHT_PHASE_FIRST_EVENT) {

#if (PARACHUTE)
    bool secondPqdTriggered = false;

  #if (Parachute_SecondNormalEvent)
    if (parachuteSecondNormal.getPqdHeight() != 0.0f) {
      secondPqdTriggered = true;
    }
  #endif

  #if (Parachute_SecondBackupEvent)
    if (parachuteSecondBackup.getPqdHeight() != 0.0f) {
      secondPqdTriggered = true;
    }
  #endif

    if (secondPqdTriggered) {
      flightState.phase = FLIGHT_PHASE_SECOND_EVENT;
      flightState.secondEventTriggered = true;
      flightState.phaseTimestampMs = sensorData.timestampMs;

      flightState.landingReferenceAltitudeM = currentAltitudeM;
      flightState.lastLandingCheckMs = sensorData.timestampMs;
    }
#endif

  }

  // SECOND_EVENT -> LANDED
  else if (flightState.phase == FLIGHT_PHASE_SECOND_EVENT) {
    float altitudeVariationM =
      fabsf(currentAltitudeM - flightState.landingReferenceAltitudeM);

    if (altitudeVariationM <= 10.0f) {
      if ((sensorData.timestampMs - flightState.lastLandingCheckMs) >= 10000UL) {
        flightState.phase = FLIGHT_PHASE_LANDED;
        flightState.landedDetected = true;
        flightState.phaseTimestampMs = sensorData.timestampMs;
      }
    } else {
      flightState.landingReferenceAltitudeM = currentAltitudeM;
      flightState.lastLandingCheckMs = sensorData.timestampMs;
    }
  }

  // LANDED
  else if (flightState.phase == FLIGHT_PHASE_LANDED) {
    flightState.landedDetected = true;
  }
}

//------------------------------ SYSTEM STATE UPDATE ------------------------------//
void updateSystemStatesFromSensors(const SensorData &sensorData) {
  SystemStates localStates = {};
  copySystemStates(localStates);

#if (BARO)
  localStates.barometerRuntimeOk = sensorData.barometerAvailable;
#endif
#if (URD_APP_SIMULATION_MODE)
  localStates.simulationRuntimeOk = sensorData.simulationAvailable;
#endif
#if (ACCEL)
  localStates.accelerometerRuntimeOk = sensorData.accelerometerAvailable;
#endif
#if (GYRO)
  localStates.gyroscopeRuntimeOk = sensorData.gyroscopeAvailable;
#endif
#if (MAG)
  localStates.magnetometerRuntimeOk = sensorData.magnetometerAvailable;
#endif
#if (HUMIDITY)
  localStates.humidityRuntimeOk = sensorData.humidityAvailable;
#endif
#if (GPS_MODE)
  localStates.gpsRuntimeOk = sensorData.gpsAvailable;
#endif
#if (BATTERY_MONITOR)
  localStates.batteryRuntimeOk = sensorData.batteryState;
#endif
#if (ENABLE_3D)
  localStates.orientationRuntimeOk = sensorData.orientationAvailable;
#endif

  updateSystemStates(localStates);
}

void updateSystemStatesFromCommunication() {
  SystemStates localStates = {};
  copySystemStates(localStates);

#if (STORAGE_MODE)
  localStates.storageRuntimeOk = sdWriteOk;
#endif

  updateSystemStates(localStates);
}
//------------------------------ 3D ORIENTATION UPDATE ------------------------------//
#if (ENABLE_3D)
void updateOrientation(SensorData &data) {
  if (!data.accelerometerAvailable || !data.gyroscopeAvailable) {
    data.orientationAvailable = false;
    return;
  }

  #if (MAG)
    orientation.update(
      data.gyroFilteredX,
      data.gyroFilteredY,
      data.gyroFilteredZ,
      data.accelFilteredX,
      data.accelFilteredY,
      data.accelFilteredZ,
      data.magFilteredX,
      data.magFilteredY,
      data.magFilteredZ
    );
    data.orientationAvailable = true;
  #else
    orientation.updateIMU(
      data.gyroFilteredX,
      data.gyroFilteredY,
      data.gyroFilteredZ,
      data.accelFilteredX,
      data.accelFilteredY,
      data.accelFilteredZ
    );
    data.orientationAvailable = true;
  #endif

  float rawRollDeg = orientation.getRoll();
  float rawPitchDeg = orientation.getPitch();
  float rawYawDeg = orientation.getYaw();

  uint32_t nowMs = data.timestampMs;

  // Phase 1: warm-up / convergence
  if (!orientationWarmupDone) {
    if ((nowMs - orientationStartTimeMs) >= ORIENTATION_CONVERGENCE_TIME_MS) {
      orientationWarmupDone = true;
    }

    data.orientationConverged = false;
    data.orientationOffsetReady = false;
    data.rollDeg = 0.0f;
    data.pitchDeg = 0.0f;
    data.yawDeg = 0.0f;
    return;
  }

  // Phase 2: collect offset samples
  if (!orientationOffsetsReady) {
    if (orientationOffsetSampleCount < ORIENTATION_OFFSET_SAMPLES) {
      rollOffsetSumDeg += rawRollDeg;
      pitchOffsetSumDeg += rawPitchDeg;
      yawOffsetSumDeg += rawYawDeg;
      orientationOffsetSampleCount++;
    }

    if (orientationOffsetSampleCount >= ORIENTATION_OFFSET_SAMPLES) {
      rollOffsetDeg = rollOffsetSumDeg / (float)orientationOffsetSampleCount;
      pitchOffsetDeg = pitchOffsetSumDeg / (float)orientationOffsetSampleCount;
      yawOffsetDeg = yawOffsetSumDeg / (float)orientationOffsetSampleCount;
      orientationOffsetsReady = true;
    }

    data.orientationOffsetReady = false;
    data.rollDeg = 0.0f;
    data.pitchDeg = 0.0f;
    data.yawDeg = 0.0f;
    return;
  }

  // Phase 3: publish corrected angles
  data.orientationOffsetReady = true;

  if (data.orientationAvailable) {
    data.rollDeg = rawRollDeg - rollOffsetDeg;
    data.pitchDeg = rawPitchDeg - pitchOffsetDeg;
    data.yawDeg = rawYawDeg - yawOffsetDeg;
    // data.yawDeg = normalizeAngle180(rawYawDeg - yawOffsetDeg);
  } else {
    data.rollDeg  = 0.0f;
    data.pitchDeg = 0.0f;
    data.yawDeg   = 0.0f;
  }
}
#endif

//--------------------------------------------------------- FLIGHT CONTROL FUNCTIONS ---------------------------------------------------------
#if (APOGEE_BARO || URD_APP_SIMULATION_MODE)

void updateWufLock(const float currentAltitudeM, const uint32_t currentTimeMs, FlightEventsData &flightData) {
  #if (WUF_PROTECTION)
    if (waitUntilFlightActive) {
      if ((currentAltitudeM > WUF_HEIGHT) || (currentAltitudeM < (-1.0f * WUF_HEIGHT))) {
        waitUntilFlightActive = false;

        #if (RESTART_PROTECTION)
          flightActive = true;
          #if defined(PLACA_ESSENTIALS_V1) || defined(PLACA_MAIN_PTH) || defined(PLACA_MAIN_SMD) || defined(PLACA_MAIN_IC)
            prefs.putBool("flightActive", flightActive);
          #endif
        #endif

        #if (TRANSONIC_PROTECTION)
          transonicProtectionActive = true;
          transonicStartTimeMs = currentTimeMs;
        #endif
      }
    }

    flightData.wufLockActive = waitUntilFlightActive;
  #endif
}

void updateTransonicLock(const uint32_t currentTimeMs, FlightEventsData &flightData) {
  #if (TRANSONIC_PROTECTION)
    if (transonicProtectionActive) {
      if ((currentTimeMs - transonicStartTimeMs) >= TIMER_TRANS) {
        transonicProtectionActive = false;
      }
    }

    flightData.transonicLockActive = transonicProtectionActive;
  #endif
}

bool areFlightLocksReleased() {
  #if (WUF_PROTECTION)
    if (waitUntilFlightActive) {
      return false;
    }
  #endif

  #if (TRANSONIC_PROTECTION)
    if (transonicProtectionActive) {
      return false;
    }
  #endif

  return true;
}

void processFlightEvents(const SensorData &sensorData, FlightEventsData &flightData) {
  flightData.timestampMs = sensorData.timestampMs;
  
  // if (!sensorData.barometerAvailable) {
  //   return;
  // } // Future emergency state maybe?

  uint32_t currentTimeMs = flightData.timestampMs;

  #if (BARO)
  flightData.inputAltitudeM = sensorData.barometerFilteredAltitudeM;
  float currentAltitudeM = flightData.inputAltitudeM;
  #elif (URD_APP_SIMULATION_MODE)
  flightData.inputAltitudeM = sensorData.simulationAltitudeM;
  float currentAltitudeM = flightData.inputAltitudeM;
  #endif

  updateWufLock(currentAltitudeM, currentTimeMs, flightData);
  updateTransonicLock(currentTimeMs, flightData);

  if (!areFlightLocksReleased()) {
    flightData.descending = false;
    flightData.apogeeDetected = false;
    return;
  }

  apogeeBaro.maxAltitude(currentAltitudeM);
  apogeeBaro.deteccaoApogeu(currentAltitudeM, 0);

  flightData.descending = apogeeBaro.getEstaDescendo();
  flightData.apogeeDetected = apogeeBaro.getEstaDescendo();
  flightData.apogeeAltitudeM = apogeeBaro.getApogeuHeight();
  flightData.apogeeTimeMs = apogeeBaro.getApogeuTime();
  flightData.maxTrackedAltitudeM = apogeeBaro.getMaxAltitude();

  #if (Parachute_FirstNormalEvent)
    parachuteFirstNormal.ativarParaquedas(currentAltitudeM, currentTimeMs, apogeeBaro.getEstaDescendo());
    flightData.parachuteFirstNormalData = parachuteFirstNormal.getData();
    flightData.parachuteFirstNormalAltitudeM = parachuteFirstNormal.getPqdHeight();
    flightData.parachuteFirstNormalTimeMs = parachuteFirstNormal.getPqdTime();
    #if (HEAL)
      flightData.parachuteFirstNormalContinuity = parachuteFirstNormal.getInfo();
    #endif
  #endif

  #if (Parachute_FirstBackupEvent)
    parachuteFirstBackup.ativarParaquedas(currentAltitudeM, currentTimeMs, apogeeBaro.getEstaDescendo());
    flightData.parachuteFirstBackupData = parachuteFirstBackup.getData();
    flightData.parachuteFirstBackupAltitudeM = parachuteFirstBackup.getPqdHeight();
    flightData.parachuteFirstBackupTimeMs = parachuteFirstBackup.getPqdTime();
    #if (HEAL)
      flightData.parachuteFirstBackupContinuity = parachuteFirstBackup.getInfo();
    #endif
  #endif

  #if (Parachute_SecondNormalEvent)
    parachuteSecondNormal.ativarParaquedas(currentAltitudeM, currentTimeMs, apogeeBaro.getEstaDescendo());
    flightData.parachuteSecondNormalData = parachuteSecondNormal.getData();
    flightData.parachuteSecondNormalAltitudeM = parachuteSecondNormal.getPqdHeight();
    flightData.parachuteSecondNormalTimeMs = parachuteSecondNormal.getPqdTime();
    #if (HEAL)
      flightData.parachuteSecondNormalContinuity = parachuteSecondNormal.getInfo();
    #endif
  #endif

  #if (Parachute_SecondBackupEvent)
    parachuteSecondBackup.ativarParaquedas(currentAltitudeM, currentTimeMs, apogeeBaro.getEstaDescendo());
    flightData.parachuteSecondBackupData = parachuteSecondBackup.getData();
    flightData.parachuteSecondBackupAltitudeM = parachuteSecondBackup.getPqdHeight();
    flightData.parachuteSecondBackupTimeMs = parachuteSecondBackup.getPqdTime();
    #if (HEAL)
      flightData.parachuteSecondBackupContinuity = parachuteSecondBackup.getInfo();
    #endif
  #endif
}
#endif

//=================================================================================== TASKs ===================================================================================
//--------------------------------------------------------- SENSOR TASK ---------------------------------------------------------
#if (SENSORS_ON)
#ifndef SENSOR_TASK_PERIOD_MS
#define SENSOR_TASK_PERIOD_MS ((uint32_t)(1000.0f / SENSOR_TASK_FREQUENCY_HZ))
#define SENSOR_DEBUG_PRINT_PERIOD_MS ((uint32_t)(1000.0f / SENSOR_DEBUG_PRINT_PERIOD_HZ))
#endif


void taskSensors(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    SensorData localData = {};
    localData.timestampMs = millis();
    
    if (lastTimestampMs != 0) {
      uint32_t dtMs = localData.timestampMs - lastTimestampMs;
      if (dtMs > 0) {
        localData.frequencyHz = (uint32_t)(1000.0f / dtMs);
      } else {
        localData.frequencyHz = 0;
      }
    } else {
      localData.frequencyHz = 0;
    }

    lastTimestampMs = localData.timestampMs;

    //------------------------------ BAROMETER ------------------------------//
    #if (BARO)
      readBarometer(localData);
    #endif

    //------------------------------ SIMULATION ------------------------------//
    #if (URD_APP_SIMULATION_MODE)
      readSimulation(localData);
    #endif

    //------------------------------ ACCELEROMETER ------------------------------//
    #if (ACCEL)
      readAccelerometer(localData);
    #endif

    //------------------------------ GYROSCOPE ------------------------------//
    #if (GYRO)
      readGyroscope(localData);
    #endif

    //------------------------------ MAGNETOMETER ------------------------------//
    #if (MAG)
      readMagnetometer(localData);
    #endif

    //------------------------------ HUMIDITY ------------------------------//
    #if (HUMIDITY)
      readHumiditySensor(localData);
    #endif

    //------------------------------ GPS ------------------------------//
    #if (GPS_MODE)
      readGps(localData);
    #endif

    //------------------------------ BATTERY ------------------------------//
    #if (BATTERY_MONITOR)
      readBatteryMonitor(localData);
    #endif

    //------------------------------ 3D ORIENTATION ------------------------------//
    #if (ENABLE_3D)
      updateOrientation(localData);
    #endif

    updateSensorData(localData);

    //------------------------------ FLIGHT EVENTS ------------------------------//
    FlightEventsData localFlightEvents = {};
    copyFlightEventsData(localFlightEvents);

    //------------------------------ FLIGHT STATES ------------------------------//
    FlightState localFlightState = {};
    copyFlightState(localFlightState);

    updateFlightStateFromSensors(localData, localFlightEvents, localFlightState);

    updateFlightState(localFlightState);

    //------------------------------ SYSTEM STATES ------------------------------//
    updateSystemStatesFromSensors(localData);

    //------------------------------ DEBUG PRINTS ------------------------------//
#if (DEBUG_PRINTS)
  static bool debugHeaderPrinted = false;
  static uint32_t lastDebugPrintMs = 0;

  if (!debugHeaderPrinted) {
    String header = "t_s\tfreq_hz";

    #if (BARO)
      header += "\tbaro_temp_c\tbaro_press_pa\tbaro_alt_m\tbaro_alt_filt_m";
    #endif

    #if (URD_APP_SIMULATION_MODE)
      header += urdApp.getHeaderString();
    #endif

    #if (ACCEL)
      header += "\taccel_x\taccel_y\taccel_z";
    #endif

    #if (GYRO)
      header += "\tgyro_x\tgyro_y\tgyro_z";
    #endif

    #if (MAG)
      header += "\tmag_x\tmag_y\tmag_z";
    #endif

    #if (HUMIDITY)
      header += "\thum_temp_c\thum_rh";
    #endif

    #if (GPS_MODE)
      header += "\tgps_lat\tgps_lon\tgps_alt_m\tgps_speed_kmph\tgps_sat\tgps_hdop";
    #endif

    #if (BATTERY_MONITOR)
      header += "\tbattery_state\tbattery_voltage\tbattery_percent";
    #endif

    #if (ENABLE_3D)
      header += "\troll_deg\tpitch_deg\tyaw_deg";
    #endif

    Serial.println(header);
    debugHeaderPrinted = true;
  }

  if ((localData.timestampMs - lastDebugPrintMs) >= SENSOR_DEBUG_PRINT_PERIOD_MS) {
    String debugLine = "";

    debugLine += String(localData.timestampMs / 1000.0f, 3) + '\t';
    debugLine += String(localData.frequencyHz);

    #if (BARO)
      if (localData.barometerAvailable) {
        debugLine +=  String(localData.barometerTemperatureC, 2) + '\t';
        debugLine += String(localData.barometerPressurePa, 2) + '\t';
        debugLine += String(localData.barometerAltitudeM, 2) + '\t';
        debugLine += String(localData.barometerFilteredAltitudeM, 2) + '\t';
      } else {
        debugLine += "NAN\tNAN\tNAN\tNAN\t";
      }
    #endif

    #if (URD_APP_SIMULATION_MODE)
    if (localData.simulationAvailable) {
      debugLine += urdApp.getDataString();
    } else {
      debugLine += "NAN\tNAN\t";
    }
    #endif

    #if (ACCEL)
      if (localData.accelerometerAvailable) {
        debugLine += String(localData.accelX, 3) + '\t';
        debugLine += String(localData.accelY, 3) + '\t';
        debugLine += String(localData.accelZ, 3) + '\t';
      } else {
        debugLine += "NAN\tNAN\tNAN\t";
      }
    #endif

    #if (GYRO)
      if (localData.gyroscopeAvailable) {
        debugLine += String(localData.gyroX, 3) + '\t';
        debugLine += String(localData.gyroY, 3) + '\t';
        debugLine += String(localData.gyroZ, 3) + '\t';
      } else {
        debugLine += "NAN\tNAN\tNAN\t";
      }
    #endif

    #if (MAG)
      if (localData.magnetometerAvailable) {
        debugLine += String(localData.magX, 3) + '\t';
        debugLine += String(localData.magY, 3) + '\t';
        debugLine += String(localData.magZ, 3) + '\t';
      } else {
        debugLine += "NAN\tNAN\tNAN\t";
      }
    #endif

    #if (HUMIDITY)
      if (localData.humidityAvailable) {
        debugLine += String(localData.humidityTemperatureC, 2) + '\t';
        debugLine += String(localData.humidityRelativePercent, 2) + '\t';
      } else {
        debugLine += "NAN\tNAN\t";
      }
    #endif

    #if (GPS_MODE)
      if (localData.gpsAvailable) {
        debugLine += String(localData.gpsLatitudeDeg, 6) + '\t';
        debugLine += String(localData.gpsLongitudeDeg, 6) + '\t';
        debugLine += String(localData.gpsAltitudeM, 2) + '\t';
        debugLine += String(localData.gpsSpeedKmph, 2) + '\t';
        debugLine += String(localData.gpsSatellites) + '\t';
        debugLine += String(localData.gpsHdop, 1) + '\t';
      } else {
        debugLine += "NAN\tNAN\tNAN\tNAN\tNAN\tNAN\t";
      }
    #endif

    #if (BATTERY_MONITOR)
      if (localData.batteryState) {
        debugLine += String(localData.batteryState ? 1 : 0) + '\t';
        debugLine += String(localData.batteryVoltage, 2) + '\t';
        debugLine += String(localData.batteryPercent, 1);
      } else {
        debugLine += "NAN\tNAN\tNAN\t";
      }
    #endif

    #if (ENABLE_3D)
      if (localData.orientationAvailable) {
        debugLine += String(localData.rollDeg, 2) + '\t';
        debugLine += String(localData.pitchDeg, 2) + '\t';
        debugLine += String(localData.yawDeg, 2) + '\t';
      } else {
        debugLine += "NAN\tNAN\tNAN\t";
      }
    #endif

    Serial.println(debugLine);
    lastDebugPrintMs = localData.timestampMs;
  }
#endif



    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
  }
}
#endif

//--------------------------------------------------------- COMMUNICATION TASK ---------------------------------------------------------
#if (COMMUNICATION_ON)
#ifndef COMMUNICATION_TASK_PERIOD_MS
#define COMMUNICATION_TASK_PERIOD_MS 20
#endif

void taskCommunication(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    SensorData localSensorData = {};
    FlightEventsData localFlightData = {};

    copySensorData(localSensorData);

    #if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
      copyFlightEventsData(localFlightData);
    #endif

    #if (LORA_MODE)
      String loraDataString = buildLoraDataString(localSensorData, localFlightData);
    #endif

    #if (NORMAL_SERIAL_PRINT_MODE || STORAGE_MODE)
      String dataString = buildDataString(localSensorData, localFlightData);
    #endif

    #if (URD_APP_SERIAL)
      String dataStringUrdApp = buildDataStringUrdApp(localSensorData, localFlightData);
    #endif

    printCommunicationHeadersIfNeeded();

    uint32_t nowMs = millis();

#if (NORMAL_SERIAL_PRINT_MODE)
    if ((nowMs - lastSerialOutputMs) >= SERIAL_OUTPUT_INTERVAL_MS) {
      Serial.println(dataString);
      lastSerialOutputMs = nowMs;
    }
#endif

#if (URD_APP_SERIAL)
    #if (URD_APP_SIMULATION_MODE || URD_APP_GS_FLIGHT_COMPUTER_SIMULATION_MODE)
    if ((nowMs - lastUrdAppOutputMs) >= SERIAL_OUTPUT_INTERVAL_MS) {
      Serial.println(dataStringUrdApp);
      lastUrdAppOutputMs = nowMs;
    }
    #endif
#endif

#if (STORAGE_MODE && SD_CARD)
    if ((nowMs - lastSdOutputMs) >= SD_SAVING_INTERVAL_MS) {
      #if (DEBUG_PRINTS_SD)
        Serial.println("SD protection: " + String(sdDelayProtectionWhenErr));
      #endif
      if (!sdDelayProtectionWhenErr) {
        logFile = SD.open(nameSD, FILE_APPEND);
        if (logFile) {
          #if (SD_LED)
            digitalWrite(LED_PIN, HIGH);
          #endif
          logFile.println(dataString);
          logFile.close();
          sdWriteOk = true;
        } else {
          sdWriteOk = false;
          sdDelayProtectionWhenErr = true;
        }
      }
        #if (SD_LED)
          digitalWrite(LED_PIN, LOW);
        #endif
      lastSdOutputMs = nowMs;
    }

    if (sdDelayProtectionWhenErr && ((nowMs - lastSdDelayOutputMs) >= SD_ERROR_RETRY_INTERVAL_MS)) {
      sdDelayProtectionWhenErr = false;
      lastSdDelayOutputMs = nowMs;
    }

    updateSystemStatesFromCommunication();
#endif

#if (LORA_MANAGER)
    verifyLoraCommand();
#endif

#if (LORA_MODE)
    if ((nowMs - lastLoRaOutputMs) >= LORA_OUTPUT_INTERVAL_MS) {
      #if (LORA_MANAGER)
        if (!changingFrequency) 
      #endif
      {
      LoRa.println(loraDataString);
      }
      lastLoRaOutputMs = nowMs;
    }
#endif

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(COMMUNICATION_TASK_PERIOD_MS));
  }
}
#endif

//--------------------------------------------------------- FLIGHT EVENTS TASK ---------------------------------------------------------
#if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
#ifndef FLIGHT_EVENTS_TASK_PERIOD_MS
#define FLIGHT_EVENTS_TASK_PERIOD_MS 10
#endif

void taskFlightEvents(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  initFlightEvents();

  for (;;) {
    SensorData localSensorData = {};
    FlightEventsData localFlightData = {};

    #if (APOGEE_BARO || URD_APP_SIMULATION_MODE || URD_APP_SIMULATION_MODE)
    copySensorData(localSensorData);
    copyFlightEventsData(localFlightData);

    processFlightEvents(localSensorData, localFlightData);

    updateFlightEventsData(localFlightData);
    #endif

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(FLIGHT_EVENTS_TASK_PERIOD_MS));
  }
}
#endif

//--------------------------------------------------------- BUZZER TASK ---------------------------------------------------------
#if (BUZZER_INDICATOR)
void taskBuzzer(void *pvParameters) {

  for (;;) {
    FlightState localFlightState = {};
    SystemStates localSystemStates = {};

    copyFlightState(localFlightState);

    switch (localFlightState.phase) {
      case FLIGHT_PHASE_INITIALIZATION: {
        vTaskDelay(pdMS_TO_TICKS(BUZZER_STARTUP_PAUSE_MS));
        copySystemStates(localSystemStates);

        #if (URD_APP_SERIAL)
          playMorseCode(BUZZER_URD_APP_MODE_MORSE);
        #else
        bool sysStates[SYSTEM_SENSORS_ON];

        int i = 0;

        #if (BARO)
        sysStates[i++] = localSystemStates.barometerInitOk;
        playDot();
        #endif

        #if (STORAGE_MODE)
        sysStates[i++] = localSystemStates.storageInitOk;
        playDot();
        #endif

        #if (ACCEL)
        sysStates[i++] = localSystemStates.accelerometerInitOk;
        playDot();
        #endif

        #if (GYRO)
        sysStates[i++] = localSystemStates.gyroscopeInitOk;
        playDot();
        #endif

        #if (MAG)
        sysStates[i++] = localSystemStates.magnetometerInitOk;
        playDot();
        #endif

        #if (HUMIDITY)
        sysStates[i++] = localSystemStates.humidityInitOk;
        playDot();
        #endif

        #if (GPS_MODE)
        sysStates[i++] = localSystemStates.gpsInitOk;
        playDot();
        #endif



        for (uint8_t i = 0; i < SYSTEM_SENSORS_ON; i++) {
          if(sysStates[i] == true) {
            playDot();
          } else {
            playDash();
          }
        }
        #endif

        localSystemStates.startupDone = true;
        updateSystemStates(localSystemStates);

        break;
      }

      case FLIGHT_PHASE_PRE_FLIGHT: {
        vTaskDelay(pdMS_TO_TICKS(BUZZER_PREFLIGHT_PAUSE_MS));
        copySystemStates(localSystemStates);
        
        #if (URD_APP_SERIAL)
          playSirenHiLo(BUZZER_URD_APP_MODE_LOOP_FREQ_HZ);
        #else
          #if (PARACHUTE)
            #if (BUZZER_SILENT_PARACHUTE_MODE)
              #if (Parachute_SecondNormalEvent)
              if (!localSystemStates.parachuteSecondNormalHealth) {
                playMorseCode(BUZZER_SQUIB_2ND_NORMAL_MORSE);
              } 
              #endif

              #if (Parachute_FirstNormalEvent)
              if (!localSystemStates.parachuteFirstNormalHealth) {
                playMorseCode(BUZZER_SQUIB_1ST_NORMAL_MORSE);
              }
              #endif

              #if (Parachute_SecondBackupEvent)
              if (!localSystemStates.parachuteSecondBackupHealth) {
                playMorseCode(BUZZER_SQUIB_2ND_BACKUP_MORSE);
              }
              #endif

              #if (Parachute_FirstBackupEvent)
              if (!localSystemStates.parachuteFirstBackupHealth) {
                playMorseCode(BUZZER_SQUIB_1ST_BACKUP_MORSE);
              }
              #endif
            #endif
          #endif

          #if (BARO)
            if (!localSystemStates.barometerRuntimeOk) {
              playMorseCode(BUZZER_BAROMETER_MORSE);
            }
          #endif

          #if (STORAGE_MODE)
            if (!localSystemStates.storageRuntimeOk) {
              playMorseCode(BUZZER_SD_MORSE);
            }
          #endif

          #if (ACCEL)
            if (!localSystemStates.accelerometerRuntimeOk) {
              playMorseCode(BUZZER_ACCELEROMETER_MORSE);
            }
          #endif

          #if (GYRO)
            if (!localSystemStates.gyroscopeRuntimeOk) {
              playMorseCode(BUZZER_GYROSCOPE_MORSE);
            }
          #endif

          #if (MAG)
            if (!localSystemStates.magnetometerRuntimeOk) {
              playMorseCode(BUZZER_MAGNETOMETER_MORSE);
            }
          #endif

          #if (HUMIDITY)
            if (!localSystemStates.humidityRuntimeOk) {
              playMorseCode(BUZZER_HUMIDITY_MORSE);
            }
          #endif

          #if (GPS_MODE)
            if (!localSystemStates.gpsRuntimeOk) {
              playMorseCode(BUZZER_GPS_MORSE);
            }
          #endif
        #endif
        break;
      }

      case FLIGHT_PHASE_BOOST:
      case FLIGHT_PHASE_COAST:
      case FLIGHT_PHASE_FIRST_EVENT:
      case FLIGHT_PHASE_SECOND_EVENT: {
        buzzerOff();
        vTaskDelay(pdMS_TO_TICKS(500));
        break;
      } 

      case FLIGHT_PHASE_LANDED: {
        vTaskDelay(pdMS_TO_TICKS(20000));
        playMaxAltitudeAsDots(localFlightState.maxAltitudeM);

        playLandingSiren(2500);

        break;
      }

      default: {
        vTaskDelay(pdMS_TO_TICKS(500));
        break;
      }
    }
  }
}
#endif

//=================================================================================== TASK DEFINITIONS ===================================================================================
#if (SENSORS_ON)
TaskHandle_t hTaskSensors = nullptr;

void createSensorTask(BaseType_t coreId, UBaseType_t priority, uint32_t stackDepth = 8192) {
  xTaskCreatePinnedToCore(
    taskSensors,
    "TaskSensors",
    stackDepth,
    nullptr,
    priority,
    &hTaskSensors,
    coreId
  );
}
#endif

#if (COMMUNICATION_ON)
void createCommunicationTask(BaseType_t coreId, UBaseType_t priority, uint32_t stackDepth = 8192) {
  xTaskCreatePinnedToCore(
    taskCommunication,
    "TaskCommunication",
    stackDepth,
    nullptr,
    priority,
    &hTaskCommunication,
    coreId
  );
}
#endif

#if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
void createFlightEventsTask(BaseType_t coreId, UBaseType_t priority, uint32_t stackDepth = 8192) {
  xTaskCreatePinnedToCore(
    taskFlightEvents,
    "TaskFlightEvents",
    stackDepth,
    nullptr,
    priority,
    &hTaskFlightEvents,
    coreId
  );
}
#endif

#if (BUZZER_INDICATOR)
void createBuzzerTask(BaseType_t coreId, UBaseType_t priority, uint32_t stackDepth = 4096) {
  xTaskCreatePinnedToCore(
    taskBuzzer,
    "TaskBuzzer",
    stackDepth,
    nullptr,
    priority,
    &hTaskBuzzer,
    coreId
  );
}
#endif

//=================================================================================== MODULE SETUP ===================================================================================

#if (BUZZER_INDICATOR)
void setupBuzzerModule() {
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, !BUZZ_ON);
}
#endif

#if (SENSORS_ON)
void setupSensorsModule() {
  if (gSensorMutex == nullptr) {
    gSensorMutex = xSemaphoreCreateMutex();
  }

  #if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
  if (gFlightEventsMutex == nullptr) {
    gFlightEventsMutex = xSemaphoreCreateMutex();
  }
  #endif

#if ((BARO) || (ACCEL) || (GYRO) || (MAG))
  #if defined(PLACA_ESSENTIALS_V1) || defined(PLACA_MAIN_PTH) || defined(PLACA_MAIN_SMD) || defined(PLACA_MAIN_IC)
    Wire.begin(SDA_PIN, SCL_PIN);
  #else
    Wire.begin();
  #endif
#endif

#if (RESTART_PROTECTION)
  #if defined(PLACA_ESSENTIALS_V1) || defined(PLACA_MAIN_PTH) || defined(PLACA_MAIN_SMD) || defined(PLACA_MAIN_IC)
    prefs.begin("config", false);
    flightActive = prefs.getBool("flightActive", false);
  #endif
#endif
}
#endif
//=================================================================================== SETUP ===================================================================================

void setup() {
  #if (SERIAL_PRINT)
  Serial.begin(SERIAL_BAUD_RATE);
  #endif

  #if (URD_APP_SERIAL)
    urdApp.begin();
    // urdApp.initLed(LED_PIN);
    urdApp.initBuzzer(BUZZ_PIN, BUZZ_ON, BUZZ_TONE, BUZZER_FREQ);
  #endif

  #if (BUZZER_INDICATOR)
    setupBuzzerModule();

      #if (BUZZER_SILENT_MODE)
        pinMode(REAL_BUZZ_PIN, OUTPUT);
        digitalWrite(REAL_BUZZ_PIN, !REAL_BUZZ_ON);
      #endif
  #else
    pinMode(BUZZ_PIN, OUTPUT);
    digitalWrite(BUZZ_PIN, !BUZZ_ON);
  #endif

  #if (SD_LED)
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
  #endif

  delay(500); // Time delay to allow for stable power before initializing sensors and tasks 

  initFlightState();
  initSystemStates();


#if (SENSORS_ON)
  initSensorData();
#endif

#if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
  initFlightEventsData();
#endif

  initAll();

#if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
  initFlightEvents();
#endif

  #if (URD_APP_SERIAL)
    #if (URD_APP_SIMULATION_MODE)
      initSimulation();
      urdApp.startHandshake(URD_APP_SIMULATION);
      calibrateSimulation();
    #elif (URD_APP_GS_FLIGHT_COMPUTER_SIMULATION_MODE)
      urdApp.startHandshake(URD_APP_GS_FLIGHT_COMPUTER_SIMULATION);
    #endif
  #endif
  
  #if (SENSORS_ON)
  createSensorTask(0, 5);
  #endif

  #if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
  createFlightEventsTask(0, 5);
  #endif

  #if (COMMUNICATION_ON)
  createCommunicationTask(1, 3);
  #endif

  #if (BUZZER_INDICATOR)
  createBuzzerTask(1, 2);
  #endif

}

//=================================================================================== LOOP ===================================================================================

void loop() {
    static uint32_t lastPrintMs = 0;

#if (DEBUG_PRINTS)
  #if (DEBUG_PRINTS_STACK_INFORMATION)
    if (millis() - lastPrintMs >= 2000) {
    #if (SENSORS_ON)
      if (hTaskSensors != nullptr) {
        UBaseType_t sensorHighWater = uxTaskGetStackHighWaterMark(hTaskSensors);
        Serial.print(F("[STACK] Sensors free words: "));
        Serial.println(sensorHighWater);
      }
    #endif

    #if (COMMUNICATION_ON)
      if (hTaskCommunication != nullptr) {
        UBaseType_t commHighWater = uxTaskGetStackHighWaterMark(hTaskCommunication);
        Serial.print(F("[STACK] Communication free words: "));
        Serial.println(commHighWater);
      }
    #endif

    #if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
    if (hTaskFlightEvents != nullptr) {
      UBaseType_t flightEventsHighWater = uxTaskGetStackHighWaterMark(hTaskFlightEvents);
      Serial.print(F("[STACK] FlightEvents free words: "));
      Serial.println(flightEventsHighWater);
    }
    #endif

      lastPrintMs = millis();
    }
  #endif
#endif
  vTaskDelay(pdMS_TO_TICKS(1000));
}