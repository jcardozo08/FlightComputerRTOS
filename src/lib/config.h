//======================================================= CONFIGURATION SYSTEM =======================================================
// Activate/deactivate sensors and modules of the embedded system. Each macro controls the inclusion of a functionality.

//------------------------------------------------- BOARD SELECTION -------------------------------------------------//
// Leave only ONE board active by uncommenting the corresponding line. This will ensure that the correct pinout and configurations are applied for your specific hardware setup.

// #define PLACA_MAIN_OCTA_PTH
// #define PLACA_ESSENTIALS_V1
#define PLACA_MAIN_PTH
// #define PLACA_MAIN_SMD
// #define PLACA_MAIN_IC

//------------------------------ MAIN SENSORS ------------------------------//
#define SENSORS_ON 1 // Master switch for all sensors (1 to activate, 0 to deactivate)
  #define BARO (SENSORS_ON && 1)          // Activate barometer
  #define ACCEL (SENSORS_ON && 1)         // Activate accelerometer
  #define GYRO (SENSORS_ON && 1)          // Activate gyroscope
  #define MAG (SENSORS_ON && 1)           // Activate magnetometer
  #define HUMIDITY (SENSORS_ON && 0)      // Activate temperature and humidity sensor DHT22

#define BATTERY_MONITOR (ARDUINO_ARCH_ESP32 && 0) // Activate voltage measurement (only for PLACA MAIN SMD and PLACA MAIN IC, only esp's family)
//------------------------------ FILTERS ------------------------------//
#define BARO_FILTERS (BARO && 1)  // Activate filters for barometer
  #define KALMAN_MovingAverage_BARO (BARO_FILTERS && 0)   // Switch between Kalman filter (1) or moving average (0) for barometer

#define ACCEL_FILTERS (ACCEL && 1) // Activate filters for accelerometer
  #define KALMAN_MovingAverage_ACCEL (ACCEL_FILTERS && 0)   // Switch between Kalman filter (1) or moving average (0) for accelerometer

#define GYRO_FILTERS (GYRO && 1)  // Activate filters for gyroscope
  #define KALMAN_MovingAverage_GYRO (GYRO_FILTERS && 0)   // Switch between Kalman filter (1) or moving average (0) for gyroscope

#define MAG_FILTERS (MAG && 1)   // Activate filters for magnetometer
  #define KALMAN_MovingAverage_MAG (MAG_FILTERS && 0)   // Switch between Kalman filter (1) or moving average (0) for magnetometer

//------------------------------ CALIBRATION ------------------------------//
#define CALIBRATION_MODE (SENSORS_ON && 1) // Activate calibration mode for sensors

#define AUTO_CALIBRATION_BARO (CALIBRATION_MODE && BARO && 0) // Switch between automatic calibration or normal calibration for barometer (Default is 0)

#define AUTO_CALIBRATION_ACCEL (CALIBRATION_MODE && ACCEL && 0) // Switch between automatic calibration or normal calibration for accelerometer
#define ACCEL_FULL_CALIBRATION_MODE (ACCEL && 1) // Switch between full calibration mode (calibrate each axis separately) or simplified calibration mode (calibrate all axes together) for accelerometer

#define AUTO_CALIBRATION_GYRO (CALIBRATION_MODE && GYRO && 0) // Switch between automatic calibration or normal calibration for gyroscope

#define AUTO_CALIBRATION_MAG (CALIBRATION_MODE && MAG && 0) // Switch between automatic calibration or normal calibration for magnetometer

//------------------------------ FLIGHT CONTROL ------------------------------//
#define WUF_PROTECTION 1  // "Wait Until Flight Condition" - Wait for Flight Condition before starting apogee detection and parachute deployment
#define TRANSONIC_PROTECTION 1 // Transonic protection - Avoid parachute deployment during transonic phase of the flight
#define RESTART_PROTECTION 1 // Restart protection - Avoid deployment problems in case of unexpected system reset during flight

//------------------------------ APOGEE FUNCTIONS ------------------------------//
#define APOGEE_BARO (BARO && 1)      // Apogee detection based on barometer readings
  #define EMERGENCY_BARO_MODE (APOGEE_BARO && 0) // Emergency barometer mode when barometer stops responding during flight (Not implemented yet) 
#define APOGEE_IMU (ACCEL && 0)   // Apogee detection based on IMU readings (Not implemented yet)

//------------------------------ COMMUNICATION ------------------------------//
#define COMMUNICATION_ON 1 // Master switch for all communication modules (1 to activate, 0 to deactivate)
  #define SERIAL_PRINT (COMMUNICATION_ON && 1)  // Enable serial output for debugging and monitoring
    #define NORMAL_SERIAL_PRINT_MODE (SERIAL_PRINT && 1) // Turn on normal serial print mode
    #define ESSENTIALS_ONLY_SERIAL_PRINT_MODE (SERIAL_PRINT && 0) // Turn on essentials-only serial print mode
  
  #define LORA_MODE (COMMUNICATION_ON && 1)      // Activate LoRa communication
    #define LORA_ONLY_ESSENTIALS (LORA_MODE && 0) // Switch between only essentials data or all data in LoRa communication
    #define LORA_MANAGER (LORA_MODE && 1) // Activate LoRa manager for handling LoRa communication and frequency changes
      #define LORA_RESET_TO_BASE_FREQ_ON_ERROR (LORA_MANAGER && 1) // Automatically reset to base frequency if there is an error during LoRa frequency change process
      #define LORA_FREQUENCY_MODE 900 // Set the LoRa frequency mode, based on the physical module (only implemented for 900 MHz yet)
      #define LORA_MANAGER_E32 1  // Set to 1 if using the E32 LoRa module (Do not change this for now, as the E22 support is not implemented yet)
      #define LORA_MANAGER_E22 0  // Set 1 if using the E22 LoRa module (Not implemented yet)
      
  #define GPS_MODE (COMMUNICATION_ON && 0)    // Activate GPS serial communication
    #define GPS_ONLY_ESSENTIALS (GPS_MODE && 0) // Switch between only essentials data or all data in GPS communication

  #define LED_INDICATOR (COMMUNICATION_ON && 0) // Enable LED indicator for system status

  #define BUZZER_INDICATOR (COMMUNICATION_ON && 1)  // Enable buzzer for system status indication
    #define BUZZER_PASSIVE_MODE (BUZZER_INDICATOR && 1) // Switch between passive or active buzzer mode for system status indication
    #define BUZZER_SILENT_MODE (BUZZER_INDICATOR && 0) // Silent mode for buzzer (use led indicator instead of buzzer for system status indication, for night testing purposes rsrs)
    #define BUZZER_SILENT_PARACHUTE_MODE (BUZZER_INDICATOR && PARACHUTE && 1) // Silent mode for buzzer ignition continuity verification, used in the redundant board without e-matches connected. 

//------------------------------ URD APP INTERFACE ------------------------------//
#define URD_APP_SERIAL (!SERIAL_PRINT && 0) // Enable serial communication with URD App for real-time telemetry visualization and analysis
  #define URD_APP_SIMULATION_MODE (URD_APP_SERIAL && 0)    // Activate simulation mode (receive simulated sensor data instead of reading from actual sensors)
  #define URD_APP_GS_FLIGHT_COMPUTER_SIMULATION_MODE (URD_APP_SERIAL && 0)    // Activate flight computer simulation mode (simulate flight computer behavior and send data to URD App for testing purposes)

//------------------------------ PARACHUTE ------------------------------//
#define PARACHUTE ((APOGEE_BARO || APOGEE_IMU || URD_APP_SIMULATION_MODE) && 1)      // Parachute deployment based on apogee detection
  #define HEAL (PARACHUTE && 1)  // System ignitors continuity verification

#define Parachute_FirstNormalEvent (PARACHUTE && 1) // First normal event after apogee (e.g., deploy first parachute)
#define Parachute_FirstBackupEvent (PARACHUTE && 1) // First backup event after apogee (e.g., deploy first backup parachute)
#define Parachute_SecondNormalEvent (PARACHUTE && 1) // Second normal event after apogee (e.g., deploy second parachute)
#define Parachute_SecondBackupEvent (PARACHUTE && 0) // Second backup event after apogee (e.g., deploy second backup parachute)

#define PARACHUTE_TEST_MODE (PARACHUTE && 0) // Parachute deployment test mode - Deploy parachute at small predefined altitude for testing purposes 
  #define ELEVATOR_TEST_MODE (PARACHUTE_TEST_MODE && 0) // Elevator test mode - Deploy parachute at small predefined altitude for testing purposes in elevator
  #define ASPIRATOR_TEST_MODE (PARACHUTE_TEST_MODE && 0) // Aspirator test mode - Deploy parachute at small predefined altitude for testing purposes with aspirator
  #define PRESSURE_CHAMBER_TEST_MODE (PARACHUTE_TEST_MODE && 0) // Pressure chamber test mode - Deploy parachute at small predefined altitude for testing purposes in pressure chamber

//------------------------------ STORAGE ------------------------------//
#define STORAGE_MODE (COMMUNICATION_ON && 1)       // Enable storage system for flight data logging
  #define SD_CARD (STORAGE_MODE && 1)          // Activate SD card storage system
  #define EEPROM (STORAGE_MODE && 0)           // Activate EEPROM storage system
  #define SD_LED (STORAGE_MODE && 0)                // Enable LED indicator for storage status

#define FILE_NAME "SAT"  // Prefix for log file names

//------------------------------ OTHER FUNCTIONS ------------------------------//
#define VERTICAL_VELOCITY (BARO && 0)  // Vertical velocity calculation based on barometer readings (Not implemented yet)
#define ENABLE_3D ((ACCEL && GYRO) && 1) // Turn on 3D orientation estimation using Madgwick algorithm based on IMU readings
#define GRAPHICAL_MARKERS 0 // Enable graphical markers in serial output for easier visualization in arduino IDE graphic plotter (Not implemented yet)

//------------------------------ DEBUG MODE ------------------------------//
#define DEBUG_PRINTS (SERIAL_PRINT && 0) // Enable extra debug prints in serial output
  #define DEBUG_PRINTS_TASK_SENSORS (DEBUG_PRINTS && SENSORS_ON && 0) // Enable extra debug prints related to sensors in serial output
  #define DEBUG_PRINTS_TASK_COMMUNICATION (DEBUG_PRINTS && COMMUNICATION_ON && 1) // Enable extra debug prints related to communication in serial output
  #define DEBUG_PRINTS_TASK_FLIGHT_EVENTS (DEBUG_PRINTS && (APOGEE_BARO || APOGEE_IMU) && 0) // Enable extra debug prints related to flight events in serial output
  #define DEBUG_PRINTS_STACK_INFORMATION (DEBUG_PRINTS && 0) // Enable debug prints of stack information for each task in serial output
  #define DEBUG_PRINTS_SD (DEBUG_PRINTS && SD_CARD && 1) // Enable extra debug prints related to SD card operations in serial output

//======================================================= END OF CONFIGURATIONS =================================================

//======================================================= PARAMETERS =================================================

// Flight States Configuration
#define BOOST_ACCEL_THRESHOLD_MS2 20.0f
#define LANDING_STABLE_TIME_MS 10000UL
#define LANDING_ALTITUDE_WINDOW_M 30.0f

// Sensors
#define SYSTEM_SENSORS_ON (BARO + ACCEL + GYRO + MAG + HUMIDITY + GPS_MODE + STORAGE_MODE)

#if (SENSORS_ON)
  #define SENSOR_TASK_FREQUENCY_HZ 100.0f
  #define SENSOR_DEBUG_PRINT_PERIOD_HZ 10.0f
#endif

#if (BARO)
  #define BARO_STARTUP_FLUSH_READS 300 // Number of initial barometer readings to flush out during startup
  #define NUM_STARTUP_READS 1000 // Number of barometer readings to take during startup for calculating initial pressure and altitude
#endif

#if (ACCEL)
  // #define ICM20948_ACC_RANGE ICM20948_ACC_RANGE_2G // Accelerometer range for ICM20948 (2G, 4G, 8G, 16G)
#endif

#if (GYRO)
  // #define ICM20948_GYRO_RANGE ICM20948_GYRO_RANGE_250 // Gyroscope range for ICM20948 (250DPS, 500DPS, 1000DPS, 2000DPS)
#endif

#if (MAG)
  // #define AK09916_CONT_MODE AK09916_CONT_MODE_100HZ // Magnetometer continuous measurement mode for AK09916 (10Hz, 20Hz, 50Hz, 100Hz)
#endif

#if (HUMIDITY)
  #define DHTTYPE DHT22  // Tipo do sensor
#endif

#if (BATTERY_MONITOR)
  #define ADC_REFERENCE_VOLTAGE 1.1f // Reference voltage for ADC in volts (1.1V for ESP32)
  #define ADC_RESOLUTION 4095.0f
  #define BATTERY_DIVIDER_R1 1000000.0f // Resistor R1 value in ohms for voltage divider
  #define BATTERY_DIVIDER_R2 1000000.0f // Resistor R2 value in ohms for voltage divider
  #define BATTERY_EMPTY_VOLTAGE 3.3f
  #define BATTERY_FULL_VOLTAGE 4.25f
#endif

// Filters
#if (KALMAN_MovingAverage_BARO)

#else
  #define NUMBER_OF_FILTERS_BARO 3 // Number of moving average cascade filters for barometer
  #define FILTER_VECTOR_SIZE_BARO 10 // Vector size for moving average filter for barometer
#endif

#if (KALMAN_MovingAverage_ACCEL)

#else
  #define NUMBER_OF_FILTERS_ACCEL 3 // Number of moving average cascade filters for accelerometer
  #define FILTER_VECTOR_SIZE_ACCEL 10 // Vector size for moving average filter for accelerometer
#endif

#if (KALMAN_MovingAverage_GYRO)

#else
  #define NUMBER_OF_FILTERS_GYRO 3 // Number of moving average cascade filters for gyroscope
  #define FILTER_VECTOR_SIZE_GYRO 10 // Vector size for moving average filter for gyroscope
#endif

#if (KALMAN_MovingAverage_MAG)

#else
  #define NUMBER_OF_FILTERS_MAG 3 // Number of moving average cascade filters for magnetometer
  #define FILTER_VECTOR_SIZE_MAG 10 // Vector size for moving average filter for magnetometer
#endif


// Calibration
//--- BAROMETER CALIBRATION ---
#define BARO_RESTART_MIN_PRESSURE_PA 85000.0f
#define BARO_RESTART_MAX_PRESSURE_PA 100000.0f

//--- ACCELEROMETER SIMPLE MANUAL CALIBRATION ---
#define ACCEL_OFFSET_X   2903.147949f
#define ACCEL_OFFSET_Y   591.651978f
#define ACCEL_OFFSET_Z   -275.688477f

#define ACCEL_SCALE_X    1.0f
#define ACCEL_SCALE_Y    1.0f
#define ACCEL_SCALE_Z    1.0f

//--- GYROSCOPE SIMPLE MANUAL CALIBRATION ---

#define GYRO_OFFSET_X    -18.434000f
#define GYRO_OFFSET_Y    31.712999f
#define GYRO_OFFSET_Z    -21.400000f

#define GYRO_SCALE_X     1.0f
#define GYRO_SCALE_Y     1.0f
#define GYRO_SCALE_Z     1.0f

//--- ACCELEROMETER FULL MANUAL CALIBRATION (MIN/MAX) ---
#define ACCEL_MIN_X   -13220.000000f
#define ACCEL_MAX_X   17056.000000f
#define ACCEL_MIN_Y   -4196.000000f
#define ACCEL_MAX_Y   15256.000000f
#define ACCEL_MIN_Z   -6088.000000f
#define ACCEL_MAX_Z   32767.000000f

// //--- MAGNETOMETER SIMPLE MANUAL CALIBRATION ---
#define MAG_OFFSET_X     0.0f
#define MAG_OFFSET_Y     0.0f
#define MAG_OFFSET_Z     0.0f

#define MAG_SCALE_X      1.0f
#define MAG_SCALE_Y      1.0f
#define MAG_SCALE_Z      1.0f

// Flight control
#if (WUF_PROTECTION) // Altitude threshold for Wait Until Flight (WUF) condition in meters
#if (ELEVATOR_TEST_MODE)
#define WUF_HEIGHT 10 
#elif (ASPIRATOR_TEST_MODE)
#define WUF_HEIGHT 25
#elif (PRESSURE_CHAMBER_TEST_MODE)
#define WUF_HEIGHT 50
#else
#define WUF_HEIGHT 50
#endif
#endif

#if (TRANSONIC_PROTECTION) // Time threshold for transonic flight protection in milliseconds
#if (ELEVATOR_TEST_MODE)
#define TIMER_TRANS 3000 // ms
#elif (ASPIRATOR_TEST_MODE)
#define TIMER_TRANS 5000 // ms
#elif (PRESSURE_CHAMBER_TEST_MODE)
#define TIMER_TRANS 7000 // ms
#else
#define TIMER_TRANS 10000 // ms
#endif
#endif

// Apogee functions
#if (APOGEE_BARO || URD_APP_SIMULATION_MODE)
  #define APOGEE_DETECTION_THRESHOLD 0.5 // Percentage threshold for apogee detection based on barometer readings
#endif

#if (PARACHUTE)
  #define ACTIVE_GPIO_TIME 200 // Duration of active GPIO status in parachute deployment in miliseconds

  #define FIRST_NORMAL_EVENT_DELAY 0 // First normal event delay after apogee in milliseconds
  #define FIRST_BACKUP_EVENT_DELAY 5000 // First backup event delay after apogee in milliseconds
  #define SECOND_NORMAL_EVENT_DELAY 0 // Second normal event delay after apogee in milliseconds
  #define SECOND_BACKUP_EVENT_DELAY 0 // Second backup event delay after apogee in milliseconds

#if (ELEVATOR_TEST_MODE || ASPIRATOR_TEST_MODE || PRESSURE_CHAMBER_TEST_MODE)
  #define ACTIVATION_HEIGHT_FIRST_NORMAL_EVENT 0 // Height threshold for first normal event after apogee in meters (0 for immediate activation after apogee)
  #define ACTIVATION_HEIGHT_FIRST_BACKUP_EVENT 0 // Height threshold for first backup event after apogee in meters (0 for immediate activation after apogee)
  #define ACTIVATION_HEIGHT_SECOND_NORMAL_EVENT 25 // Height threshold for second normal event after apogee in meters (0 for immediate activation after apogee)
  #define ACTIVATION_HEIGHT_SECOND_BACKUP_EVENT 20 // Height threshold for second backup event after apogee in meters (0 for immediate activation after apogee)
#else
  #define ACTIVATION_HEIGHT_FIRST_NORMAL_EVENT 0 // Height threshold for first normal event after apogee in meters (0 for immediate activation after apogee)
  #define ACTIVATION_HEIGHT_FIRST_BACKUP_EVENT 0 // Height threshold for first backup event after apogee in meters (0 for immediate activation after apogee)
  #define ACTIVATION_HEIGHT_SECOND_NORMAL_EVENT 450 // Height threshold for second normal event after apogee in meters (0 for immediate activation after apogee)
  #define ACTIVATION_HEIGHT_SECOND_BACKUP_EVENT 400 // Height threshold for second backup event after apogee in meters (0 for immediate activation after apogee)
#endif
#endif

// Communication
#if (SERIAL_PRINT || URD_APP_SERIAL)
#define SERIAL_BAUD_RATE 115200 // Serial print baud rate
#define SERIAL_OUTPUT_INTERVAL_MS 20 // Serial print interval in milliseconds
#endif

#if (LORA_MODE) 
#define LORA_BAUD_RATE 9600 // LoRa serial communication baud rate
#define LORA_OUTPUT_INTERVAL_MS 2000 // LoRa output interval in milliseconds

#if (LORA_MANAGER)
#define LORA_BASE_CHAN 41 // LoRa frequency in MHz
#define LORA_BASE_ADDH 0x00 // LoRa address high byte
#define LORA_BASE_ADDL 0x17 // LoRa address low byte
#define LORA_CHANGE_TIMEOUT_MS 10000 // Timeout for LoRa frequency change process in milliseconds

#define LORA_CHANGE_REQUEST_FLIGHT "MUD4R_FR3Q_PFV."
#define LORA_CHANGE_REQUEST_CONFIRMATION_FLIGHT "CTZ_FR3Q."
#define LORA_CHANGE_REQUEST_FINAL_CONFIRMATION_FLIGHT "1SSO_MSM"
#define LORA_CHANGE_FINAL_DONE_CONFIRMATION_GS_FLIGHT "MUD0U_MSM"
#define LORA_CHANGE_FINAL_DONE_CONFIRMATION_FC_FLIGHT "JUR0_JUR4D1NH0"
#define LORA_CHANGE_FINAL_DONE_COMMUNICATION_FC_FLIGHT "B04"
#define LORA_CHANGE_DONE_ERROR_FLIGHT "MUD4R_ERRO"
#endif

#endif


#if (GPS_MODE)
#define GPS_BAUD_RATE 9600 // GPS serial communication baud rate
#endif

#if (LED_INDICATOR)
#define LED_DELAY 5000 // LED indicator delay in ms
#endif

#if (BUZZER_INDICATOR)
#define BUZZER_DELAY 15000 // Buzzer indicator delay in ms
  #define BUZZER_TASK_FREQUENCY_HZ 10.0f
  #define BUZZER_STATUS_INTERVAL_MS 30000UL

  #define BUZZER_FREQ 2000 

  #define BUZZER_DOT_TIME_MS 400
  #define BUZZER_DASH_TIME_MS 1200
  #define BUZZER_SYMBOL_PAUSE_MS 300
  #define BUZZER_GROUP_PAUSE_MS 700

  #define BUZZER_LETTER_MORSE_PAUSE_MS 1000
  #define BUZZER_WORD_MORSE_PAUSE_MS 2000

  #define MAX_ALTITUDE_DIGIT_GAP_MS      500
  #define MAX_ALTITUDE_BEEP_GAP_MS   2000

  #define SIREN_PAUSE_MS 300

  #define BUZZER_STARTUP_PAUSE_MS 3000
  #define BUZZER_PREFLIGHT_PAUSE_MS 30000

  #define BUZZER_SD_MORSE                     "..."    // S
  #define BUZZER_BAROMETER_MORSE              "-..."   // B
  #define BUZZER_ACCELEROMETER_MORSE          ".-"     // A
  #define BUZZER_GYROSCOPE_MORSE              "--."    // G
  #define BUZZER_MAGNETOMETER_MORSE           "--"     // M
  #define BUZZER_HUMIDITY_MORSE               "...."   // H
  #define BUZZER_GPS_MORSE                    ".-.."   // L

  #define BUZZER_SQUIB_2ND_NORMAL_MORSE      ".-. -- -."    // R M N
  #define BUZZER_SQUIB_1ST_NORMAL_MORSE    ".-. -.. -."     // R D N
  #define BUZZER_SQUIB_2ND_BACKUP_MORSE      ".-. -- -..."  // R M B
  #define BUZZER_SQUIB_1ST_BACKUP_MORSE    ".-. -.. -..."   // R D B

  #define BUZZER_URD_APP_MODE_MORSE           "..-"  // U
  #define BUZZER_URD_APP_MODE_LOOP_FREQ_HZ     1500 // Hz

#endif

// Storage
#if (STORAGE_MODE)
  #define SD_SAVING_INTERVAL_MS 300 // SD card saving interval in milliseconds
  #define SD_ERROR_RETRY_INTERVAL_MS 5000 // Time interval to wait before retrying SD card write after an error in milliseconds
#endif

// 3D Orientation
#if (ENABLE_3D)
  #define ORIENTATION_CONVERGENCE_TIME_MS 10000UL // Time in milliseconds for the Madgwick filter to converge to a stable orientation estimation during startup
  #define ORIENTATION_OFFSET_SAMPLES 1000 // Number of initial samples to calculate orientation offset during startup
#endif


//======================================================= END OF PARAMETERS =================================================

//------------------------------------------------- BOARD NAME -------------------------------------------------//
#if defined(PLACA_MAIN_OCTA_PTH)
  #define NOME_PLACA "PLACA MAIN OCTA PTH"
#elif defined(PLACA_ESSENTIALS_V1)
  #define NOME_PLACA "ESSENTIALS V1"
#elif defined(PLACA_MAIN_PTH)
  #define NOME_PLACA "PLACA MAIN PTH"
#elif defined(PLACA_MAIN_SMD)
  #define NOME_PLACA "PLACA MAIN SMD"
#elif defined(PLACA_MAIN_IC)
  #define NOME_PLACA "PLACA MAIN IC"
#endif

//------------------------------------------------- BOARDS VALIDATIONS -------------------------------------------------//
#if (defined(PLACA_MAIN_OCTA_PTH) + defined(PLACA_ESSENTIALS_V1) + defined(PLACA_MAIN_PTH) + defined(PLACA_MAIN_SMD) + defined(PLACA_MAIN_IC)) > 1
  #error "More than one board was selected in the config."
#endif

#if !defined(PLACA_MAIN_OCTA_PTH) && !defined(PLACA_ESSENTIALS_V1) && !defined(PLACA_MAIN_PTH) && !defined(PLACA_MAIN_SMD) && !defined(PLACA_MAIN_IC)
  #error "No board was selected in the config."
#endif

#if (SD_LED && BUZZER_SILENT_MODE)
  #error "Both buzzer and LED indicators are activated. Use only one."
#endif

#if (URD_APP_SIMULATION_MODE && BARO)
  #error "URD App Simulation Mode can only be used without a barometer."
#endif

#if (URD_APP_SIMULATION_MODE && URD_APP_GS_FLIGHT_COMPUTER_SIMULATION_MODE)
  #error "Only one URD App mode can be activated at a time."
#endif

#if (URD_APP_GS_FLIGHT_COMPUTER_SIMULATION_MODE)
  #if (!APOGEE_BARO || !BARO)
  #error "URD App GS Flight Computer Simulation Mode requires BARO and APOGEE_BARO to be activated."
  #endif
#endif

//------------------------------------------------- WARNINGS -------------------------------------------------//
#if (PARACHUTE && (!APOGEE_BARO || !APOGEE_IMU))
  #warning "Parachute deployment is activated but no apogee detection method is selected. Please activate at least one apogee detection method for parachute deployment to work."
#endif

#if (ELEVATOR_TEST_MODE || ASPIRATOR_TEST_MODE || PRESSURE_CHAMBER_TEST_MODE)
  #warning "A parachute deployment test mode is activated. Please deactivate the parachute test mode before flight."
#endif
