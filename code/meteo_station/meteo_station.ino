#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h" // BMP581 library
#include "SparkFun_SCD30_Arduino_Library.h"  // SCD30 library
#include "SHTSensor.h"                      // SHT3x library
#include <SD.h>
#include <SPI.h>
#include <Adafruit_GPS.h>                   // GPS library
#include <Watchdog_t4.h>                    // Watchdog library
#include <InternalTemperature.h>            // Teensy Internal Temp

// --- Watchdog Configuration ---
WDT_T4<WDT1> wdt; 
const int WATCHDOG_TIMEOUT_SECONDS = 20; // Timeout in seconds.

// Sensor objects
SCD30 airSensor;
BMP581 pressureSensor; 
SHTSensor sht1(SHTSensor::SHT3X);

// GPS Configuration
#define GPSSerial Serial8
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false
// I2C Address for BMP581
#define I2C_ADDRESS_1 0x46 // BMP581 I2C address
// SD Card Chip Select
const int chipSelect = BUILTIN_SDCARD; // BUILTIN_SDCARD for Teensy 4.1

// --- Global variables to store the last known sensor values ---
float cpu_temp = 0.0;
float lastBmpTemp = 0.0;
float lastBmpPressure = 0.0; // In Pascals (Pa)
int   lastCo2 = 0;
float lastCo2Temp = 0.0;
float lastCo2Humidity = 0.0;
float lastShtTemp = 0.0;
float lastShtHumidity = 0.0;
float lastGpsLat = 0.0;
float lastGpsLon = 0.0;
float lastGpsAlt = 0.0;
uint16_t lastGpsYear = 0;
uint8_t  lastGpsMonth = 0;
uint8_t  lastGpsDay = 0;
uint8_t  lastGpsHour = 0;
uint8_t  lastGpsMinute = 0;
uint8_t  lastGpsSecond = 0;
uint16_t lastGpsMilliSecond = 0;
bool gpsFixAvailable = false;

// --- Global counter for log entries ---
unsigned long logEntryCounter = 0;

// --- SD Card Logging Configuration ---
const unsigned long MAX_DATA_LINES_PER_FILE = 43200; // Approx 12 hours of data at 1 line/sec
unsigned long currentDataLinesInFile = 0;     // Tracks lines in the current file
int currentLogFileNumber = 1;                 // Current log file number, e.g., 1 for LOG_1.TXT
char currentLogFileName[20];                  // Stores the current log file name
const char* CSV_HEADER = "Log_Index,CPU_TempC,BMP_TempC,BMP_PressPa,CO2_ppm,SCD30_TempC,SCD30_Hum_%,SHT_TempC,SHT_Hum_%,GPS_Lat,GPS_Lon,GPS_Alt_m,GPS_Year,GPS_Month,GPS_Day,GPS_Hour,GPS_Min,GPS_Sec,GPS_MilliSec,GPS_Fix,BMP_Health,SCD30_Health,SHT_Health";

// --- Error Handling and LED ---
bool mainLedTurnedOnError = false; // Tracks if the main LED has been turned on due to an error

// --- Sensor Health Tracking ---
unsigned long lastBmpSuccessTime = 0;
unsigned long lastScd30SuccessTime = 0;
unsigned long lastShtSuccessTime = 0;
// Timeout durations for sensors (in milliseconds)
const unsigned long SENSOR_TIMEOUT_MS_BMP_SHT = 20000; // 20 seconds
const unsigned long SENSOR_TIMEOUT_MS_SCD30 = 20000; // 20 seconds for SCD30

// --- Sensor Health Flags (will be logged) ---
bool bmpHealthOk = false;
bool scd30HealthOk = false;
bool shtHealthOk = false;

// --- START OF MODIFICATIONS: Global variables for pressure averaging ---
const int PRESSURE_AVG_WINDOW_SIZE = 60;
float pressureValuesForAvg[PRESSURE_AVG_WINDOW_SIZE];
int pressureValueCount = 0;
// --- END OF MODIFICATIONS: Global variables ---


// --- Function to turn on the main LED ---
void turnOnMainLed() {
    digitalWrite(LED_BUILTIN, HIGH);
    if (!mainLedTurnedOnError) {
        Serial.println("CRITICAL ERROR DETECTED: Main LED has been turned ON. Please check system.");
        mainLedTurnedOnError = true;
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW); // Start with LED OFF

    Serial.begin(115200);
    // while(!Serial && millis() < 4000); // Optional: wait for serial port to connect with timeout
    Serial.println("System Setup Starting...");

    Serial.print("Initializing Watchdog with timeout: ");
    Serial.print(WATCHDOG_TIMEOUT_SECONDS);
    Serial.println(" seconds.");
    WDT_timings_t config;
    config.timeout = WATCHDOG_TIMEOUT_SECONDS; // Set timeout
    config.window = 0; // Window must be 0 for this WDT_T4 version or less than timeout
    wdt.begin(config);
    wdt.feed(); // Feed watchdog

    Serial.println("Initializing SD card...");
    if (!SD.begin(chipSelect)) {
        Serial.println("SD Card failed, or not present. System Halted.");
        turnOnMainLed(); // Turn on LED to indicate critical failure
    } else {
        Serial.println("SD card initialized.");
    }
    
    Serial.println("Determining new log file name...");
    currentLogFileNumber = 1; 
    while (true) {
        sprintf(currentLogFileName, "LOG_%d.TXT", currentLogFileNumber);
        if (SD.exists(currentLogFileName)) {
            currentLogFileNumber++; 
            if (currentLogFileNumber > 999999) { 
                Serial.println("Exceeded maximum log file number search (999999). System Halted.");
                turnOnMainLed();
                // Consider halting if this state is reached
            }
        } else {
            break; 
        }
    }
    currentDataLinesInFile = 0; 
    Serial.print("Will create and write to new log file: ");
    Serial.println(currentLogFileName);

    Wire.begin();  // Main I2C bus
    Wire2.begin(); // Second I2C bus
    delay(100);    // Allow I2C to settle

    Serial.println("Initializing BMP581...");
    if (!pressureSensor.beginI2C(I2C_ADDRESS_1, Wire2)) { 
        Serial.println("SETUP ERROR: BMP581 not detected. Please check wiring.");
        turnOnMainLed();
    } else {
        Serial.println("BMP581 initialized.");
        lastBmpSuccessTime = millis();
        if (lastBmpSuccessTime == 0) lastBmpSuccessTime = 1; 
    }
    delay(100);
    writeRegister(I2C_ADDRESS_1, 0x18, 0b00000011, Wire2); 
    delay(100);
    writeRegister(I2C_ADDRESS_1, 0x31, 0b00111111, Wire2); 
    delay(100);
    writeRegister(I2C_ADDRESS_1, 0x36, 0b01111111, Wire2); 
    delay(100);
    writeRegister(I2C_ADDRESS_1, 0x30, 0b11111011, Wire2); 
    delay(100);
    writeRegister(I2C_ADDRESS_1, 0x37, 0b11110001, Wire2); 
    delay(100);

    Serial.println("Initializing SCD30...");
    if (airSensor.begin(Wire) == false){
      Serial.println("SETUP ERROR: Air sensor (SCD30) not detected. Please check wiring.");
      turnOnMainLed();
    } else {
        Serial.println("SCD30 initialized.");
        airSensor.setMeasurementInterval(2); 
        lastScd30SuccessTime = millis();
        if (lastScd30SuccessTime == 0) lastScd30SuccessTime = 1;
    }
    delay(100);

    Serial.println("Initializing SHT3x...");
    sht1.init(Wire2); 
    if (!sht1.readSample()) { 
        Serial.println("SETUP ERROR: SHT3x not detected or error reading sample on setup.");
        turnOnMainLed();
    } else {
        Serial.println("SHT3x initialized and first sample read.");
        lastShtSuccessTime = millis();
        if (lastShtSuccessTime == 0) lastShtSuccessTime = 1;
    }

    GPSSerial.begin(9600);
    GPS.begin(9600); 
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA); //PMTK_SET_NMEA_OUTPUT_RMCGGA
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    
    GPS.sendCommand(PGCMD_ANTENNA);               
    delay(1000); 

    Serial.println("Setup complete. Starting main loop.");
}

unsigned long previousLogTime = 0;
const unsigned long logInterval = 1000; // Log data every 1 second

void loop() {
    wdt.feed(); 
    unsigned long currentMillis = millis();

    while (GPSSerial.available() > 0) { 
        GPS.read();
    }

    if (GPS.newNMEAreceived()) {
        if (GPS.parse(GPS.lastNMEA())) { 
            if (GPS.fix) {
                gpsFixAvailable = true;
                lastGpsLat = GPS.latitudeDegrees;
                lastGpsLon = GPS.longitudeDegrees;
                lastGpsAlt = GPS.altitude;
                lastGpsYear = GPS.year + 2000; 
                lastGpsMonth = GPS.month;
                lastGpsDay = GPS.day;
                lastGpsHour = GPS.hour;
                lastGpsMinute = GPS.minute;
                lastGpsSecond = GPS.seconds;
                lastGpsMilliSecond = GPS.milliseconds;
            } else {
                gpsFixAvailable = false;
            }
        }
    }

    bmp5_sensor_data bmpData = {0, 0}; 
    int8_t bmpErr = pressureSensor.getSensorData(&bmpData);
    if (bmpErr == BMP5_OK) {
        lastBmpTemp = bmpData.temperature; 
        uint32_t rawPressure = readPressure(Wire2); 
        lastBmpPressure = convertRawPressure(rawPressure); 
        lastBmpSuccessTime = currentMillis; 

        // --- Averaging pressure for SCD30 offset ---
        // Check if the converted pressure is valid (e.g., greater than 0 Pa).
        // BMP5_OK ensures sensor data is valid; convertRawPressure might return 0 if its internal read failed.
        if (lastBmpPressure > 0) { 
            pressureValuesForAvg[pressureValueCount] = lastBmpPressure;
            pressureValueCount++;

            if (pressureValueCount >= PRESSURE_AVG_WINDOW_SIZE) {
                float sumOfPressures = 0.0;
                for (int i = 0; i < PRESSURE_AVG_WINDOW_SIZE; i++) {
                    sumOfPressures += pressureValuesForAvg[i];
                }
                float averagePressurePa = sumOfPressures / PRESSURE_AVG_WINDOW_SIZE;
                uint16_t averagePressureMbar = (uint16_t)(averagePressurePa / 100.0); // Convert Pa to mbar

                //Serial.print("Collected 60 pressure samples. Average (Pa): "); Serial.print(averagePressurePa);
                //Serial.print(" -> mbar for SCD30: "); Serial.println(averagePressureMbar);

                if (airSensor.setAmbientPressure(averagePressureMbar)) {
                    //Serial.println("SCD30: Ambient pressure compensation successfully set.");
                } else {
                    Serial.println("SCD30: FAILED to set ambient pressure compensation.");
                }
                
                pressureValueCount = 0; // Reset counter for the next averaging window
            }
        }
    } 

    if (airSensor.dataAvailable()) {
        lastCo2 = airSensor.getCO2();
        lastCo2Temp = airSensor.getTemperature();
        lastCo2Humidity = airSensor.getHumidity();
        lastScd30SuccessTime = currentMillis; 
    }

    if (sht1.readSample()) { 
        lastShtTemp = sht1.getTemperature();
        lastShtHumidity = sht1.getHumidity();
        lastShtSuccessTime = currentMillis; 
    } 

    if (lastBmpSuccessTime != 0 && (currentMillis - lastBmpSuccessTime > SENSOR_TIMEOUT_MS_BMP_SHT)) {
        Serial.println("LOOP ERROR: BMP581 has not updated data recently.");
        turnOnMainLed(); 
    }
    if (lastScd30SuccessTime != 0 && (currentMillis - lastScd30SuccessTime > SENSOR_TIMEOUT_MS_SCD30)) {
        Serial.println("LOOP ERROR: SCD30 has not provided data recently.");
        turnOnMainLed(); 
    }
    if (lastShtSuccessTime != 0 && (currentMillis - lastShtSuccessTime > SENSOR_TIMEOUT_MS_BMP_SHT)) {
        Serial.println("LOOP ERROR: SHT3x has not updated data recently.");
        turnOnMainLed(); 
    }

    cpu_temp = InternalTemperature.readTemperatureC();

    if (currentMillis - previousLogTime >= logInterval) {
        previousLogTime = currentMillis;

        bmpHealthOk = (lastBmpSuccessTime != 0) && ((currentMillis - lastBmpSuccessTime) <= SENSOR_TIMEOUT_MS_BMP_SHT);
        scd30HealthOk = (lastScd30SuccessTime != 0) && ((currentMillis - lastScd30SuccessTime) <= SENSOR_TIMEOUT_MS_SCD30);
        shtHealthOk = (lastShtSuccessTime != 0) && ((currentMillis - lastShtSuccessTime) <= SENSOR_TIMEOUT_MS_BMP_SHT);

        if (currentDataLinesInFile >= MAX_DATA_LINES_PER_FILE) {
            currentLogFileNumber++; 
            if (currentLogFileNumber > 999999) { 
                Serial.println("Exceeded maximum log file number (999999) during rollover. System Halted.");
                turnOnMainLed();
            }
            currentDataLinesInFile = 0; 
            sprintf(currentLogFileName, "LOG_%d.TXT", currentLogFileNumber);
            Serial.print("Max data lines reached. Rolling over to new log file: ");
            Serial.println(currentLogFileName);
        }

        char buf[400]; 
        snprintf(buf, sizeof(buf),
                "%lu,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.2f,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d",
                logEntryCounter,      
                cpu_temp,
                lastBmpTemp, lastBmpPressure, 
                lastCo2, lastCo2Temp, lastCo2Humidity, 
                lastShtTemp, lastShtHumidity,         
                gpsFixAvailable ? lastGpsLat : 0.0,   
                gpsFixAvailable ? lastGpsLon : 0.0,   
                gpsFixAvailable ? lastGpsAlt : 0.0,   
                gpsFixAvailable ? lastGpsYear : 0,    
                gpsFixAvailable ? lastGpsMonth : 0,   
                gpsFixAvailable ? lastGpsDay : 0,     
                gpsFixAvailable ? lastGpsHour : 0,    
                gpsFixAvailable ? lastGpsMinute : 0,  
                gpsFixAvailable ? lastGpsSecond : 0,  
                gpsFixAvailable ? lastGpsMilliSecond : 0,  
                gpsFixAvailable ? 1 : 0,              
                bmpHealthOk ? 1 : 0,                  
                scd30HealthOk ? 1 : 0,                
                shtHealthOk ? 1 : 0                   
        );
        
        Serial.print("DATA: "); 
        Serial.println(buf); 

        File dataFile = SD.open(currentLogFileName, FILE_WRITE);

        if (dataFile) {
            if (dataFile.size() == 0) {
                dataFile.println(CSV_HEADER);
                Serial.print("Wrote CSV header to ");
                Serial.println(currentLogFileName);
            }
            dataFile.println(buf);    
            dataFile.close();         
            currentDataLinesInFile++; 
            logEntryCounter++;        
        } else {
            Serial.print("LOOP ERROR: Error opening ");
            Serial.print(currentLogFileName);
            Serial.println(" for writing.");
            turnOnMainLed(); 
        }
    }
}

// --- Helper functions (Unchanged) ---
void writeRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t value, TwoWire &i2cPort) {
    i2cPort.beginTransmission(deviceAddress);
    i2cPort.write(registerAddress);
    i2cPort.write(value);
    i2cPort.endTransmission();
}

uint8_t readRegister(uint8_t deviceAddress, uint8_t registerAddress, TwoWire &i2cPort) {
    i2cPort.beginTransmission(deviceAddress);
    i2cPort.write(registerAddress);
    i2cPort.endTransmission(false); 
    i2cPort.requestFrom(deviceAddress, (uint8_t)1);
    if (i2cPort.available()) {
        return i2cPort.read();
    } else {
        Serial.print("Failed to read from I2C address 0x");
        Serial.print(deviceAddress, HEX);
        Serial.print(" register 0x");
        Serial.println(registerAddress, HEX);
        return 0; 
    }
}

void checkCommunication(uint8_t address, TwoWire &i2cPort) {
    i2cPort.beginTransmission(address);
    i2cPort.write(0x37); 
    i2cPort.endTransmission(false); 
    i2cPort.requestFrom(address, (uint8_t)1);
    if (i2cPort.available()) {
        uint8_t data = i2cPort.read();
        Serial.print("Status/Register 0x37 from 0x");
        Serial.print(address,HEX);
        Serial.print(": 0x");
        Serial.println(data, HEX);
    } else {
        Serial.print("Failed to read from I2C address 0x");
        Serial.println(address, HEX);
    }
}

uint32_t readPressure(TwoWire &i2cPort) {
    byte data[3]; 
    i2cPort.beginTransmission(I2C_ADDRESS_1);
    i2cPort.write(0x20);  
    i2cPort.endTransmission(false); 

    i2cPort.requestFrom(I2C_ADDRESS_1, (uint8_t)3); 

    if (i2cPort.available() == 3) {
        data[0] = i2cPort.read(); 
        data[1] = i2cPort.read(); 
        data[2] = i2cPort.read(); 
    } else {
        Serial.println("Error reading pressure data bytes from BMP581 via custom readPressure.");
        return 0; 
    }
    uint32_t pressure = ((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | data[0];
    return pressure;
}

float convertRawPressure(uint32_t rawPressure) {
    if (rawPressure == 0) return 0.0f; // Handle error from readPressure
    float pressure = (float)rawPressure / 64.0f;
    return pressure;
}