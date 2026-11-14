/**
 * @file tb.cpp
 * @brief Cookstove Data Acquisition (DAQ) Firmware - 2 Hz State Machine
 *
 * Hardware Architecture:
 *  1. Adafruit Temp/Humidity Sensor (SHT31/SHT4x) -> I2C Bus
 *  2. Analog CO Sensor -> 16-bit ADC (ADS1115)    -> Shared I2C Bus
 *  3. Particulate Matter Sensor (SPS30 / PM2.5)   -> Hardware UART
 *  4. Dual-Channel Thermocouple Daughter Board    -> SPI (2x MAX31855)
 *     - Channel 1: Firebox Temperature
 *     - Channel 2: Water Pot Temperature
 *  5. MicroSD Card Storage                        -> SPI (JSONL Logging)
 *  6. Host PC / Gateway                           -> USB Serial (JSONL Stream)
 * 
 ** AI was used to create the initial template for this document
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// Sensor Libraries (Install via PlatformIO / Arduino Library Manager)
#include <Adafruit_SHT31.h>      // Adafruit Temp/Humidity (swap for Adafruit_SHT4x if needed)
#include <Adafruit_ADS1X15.h>    // 16-bit I2C ADC (ADS1115)
#include <Adafruit_MAX31855.h>   // Thermocouple-to-Digital Daughter Board
#include <sps30.h>               // Sensirion SPS30 PM2.5 UART Driver

// ============================================================================
// 1. PIN & HARDWARE CONFIGURATION MACROS (Edit these to match your wiring)
// ============================================================================

// --- Shared I2C Bus (Temp/Humidity Sensor + 16-bit CO ADC) ---
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22
#define I2C_CLOCK_HZ            100000UL  // 100 kHz standard mode for bus reliability

// I2C Device Addresses
#define ADDR_TEMP_HUM_SENSOR    0x44      // Default Adafruit SHT31/SHT4x address
#define ADDR_CO_ADC_16BIT       0x48      // Default ADS1115 address (ADDR pin to GND)
#define CO_ADC_CHANNEL          0         // ADS1115 analog input channel (AIN0)

// --- UART Bus (PM2.5 Sensor) ---
#define PIN_PM25_UART_RX        16        // MCU RX <- PM2.5 TX
#define PIN_PM25_UART_TX        17        // MCU TX -> PM2.5 RX
#define PM25_UART_BAUD          115200    // 115200 for Sensirion SPS30 (use 9600 for PMS5003)
#define PM25_SERIAL_PORT        Serial1   // Hardware UART instance

// --- SPI Bus (Thermocouple Daughter Board + SD Card) ---
#define PIN_SPI_SCK             18
#define PIN_SPI_MISO            19
#define PIN_SPI_MOSI            23

// Chip Select (CS) Pins for SPI Peripherals
#define PIN_TC_FIREBOX_CS       5         // Thermocouple 1: Firebox temp CS
#define PIN_TC_WATER_CS         4         // Thermocouple 2: Water temp CS
#define PIN_SD_CARD_CS          15        // MicroSD card CS for JSONL storage

// --- Acquisition Timing & Data Output ---
#define USB_SERIAL_BAUD         115200    // Baud rate for Python serial readout
#define SAMPLE_RATE_HZ          2         // Target acquisition frequency (2 Hz)
#define SAMPLE_PERIOD_MS        (1000UL / SAMPLE_RATE_HZ) // 500 ms per cycle
#define JSONL_FILE_PATH         "/cookstove_daq.jsonl"

// ============================================================================
// 2. STATE MACHINE & DATA STRUCTURES
// ============================================================================

/**
 * @brief Acquisition state machine states.
 * Each sensor state reads one peripheral and immediately appends its JSON
 * fields to the active JSONL file before transitioning to the next state.
 */
enum DaqState {
    STATE_IDLE_WAIT = 0,          // Wait for next 500 ms (2 Hz) sample tick
    STATE_OPEN_JSON_RECORD,       // Open JSONL file and write opening '{' + timestamp
    STATE_READ_SAVE_TEMP_HUM,     // Read I2C Temp/Humidity -> save to JSONL
    STATE_READ_SAVE_CO_ADC,       // Read I2C 16-bit CO ADC -> save to JSONL
    STATE_READ_SAVE_PM25,         // Read UART PM2.5 sensor -> save to JSONL
    STATE_READ_SAVE_THERMOCOUPLES,// Read Firebox & Water TCs -> save to JSONL
    STATE_FINALIZE_RECORD         // Close '}\n', flush SD file, stream over USB Serial
};

// Global state tracker and timing variables
static DaqState currentState = STATE_IDLE_WAIT;
static unsigned long lastSampleTickMs = 0;

// Hardware driver instances
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_ADS1115 ads1115;
Adafruit_MAX31855 tcFirebox(PIN_TC_FIREBOX_CS);
Adafruit_MAX31855 tcWater(PIN_TC_WATER_CS);

// File handle and USB mirror buffer for the active JSONL line
File jsonlFile;
char usbJsonBuffer[512];
size_t usbBufferIdx = 0;
bool sdAvailable = false;

// ============================================================================
// 3. JSONL FORMATTING & FILE HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Appends raw text to both the SD card JSONL file and the USB mirror buffer.
 */
void writeJsonChunk(const char* text) {
    if (sdAvailable && jsonlFile) {
        jsonlFile.print(text);
    }
    size_t len = strlen(text);
    if (usbBufferIdx + len < sizeof(usbJsonBuffer) - 1) {
        memcpy(&usbJsonBuffer[usbBufferIdx], text, len);
        usbBufferIdx += len;
        usbJsonBuffer[usbBufferIdx] = '\0';
    }
}

/**
 * @brief Formats and saves a floating-point sensor field (or "null" on sensor error).
 */
void saveJsonFloatField(const char* key, float value, uint8_t decimals, bool isValid, bool leadingComma = true) {
    char chunk[64];
    char valStr[24];

    if (isValid && !isnan(value) && !isinf(value)) {
        dtostrf(value, 1, decimals, valStr);
    } else {
        strcpy(valStr, "null");
    }

    snprintf(chunk, sizeof(chunk), "%s\"%s\":%s", leadingComma ? "," : "", key, valStr);
    writeJsonChunk(chunk);
}

/**
 * @brief Formats and saves a signed integer sensor field (or "null" on sensor error).
 */
void saveJsonIntField(const char* key, int32_t value, bool isValid, bool leadingComma = true) {
    char chunk[64];
    if (isValid) {
        snprintf(chunk, sizeof(chunk), "%s\"%s\":%ld", leadingComma ? "," : "", key, (long)value);
    } else {
        snprintf(chunk, sizeof(chunk), "%s\"%s\":null", leadingComma ? "," : "", key);
    }
    writeJsonChunk(chunk);
}

// ============================================================================
// 4. SENSOR INITIALIZATION
// ============================================================================

void initPeripherals() {
    // 1. Initialize USB Serial for host Python readout
    Serial.begin(USB_SERIAL_BAUD);
    while (!Serial && millis() < 2000) {
        ; // Wait briefly for native USB CDC enumeration
    }

    // 2. Initialize shared I2C bus (Temp/Humidity + 16-bit CO ADC)
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(I2C_CLOCK_HZ);

    if (!sht31.begin(ADDR_TEMP_HUM_SENSOR)) {
        Serial.println("{\"status\":\"warn\",\"msg\":\"SHT31 Temp/Hum not detected on I2C\"}");
    }

    // Configure ADS1115 gain: GAIN_ONE = +/- 4.096V range (0.125 mV per bit)
    ads1115.setGain(GAIN_ONE);
    if (!ads1115.begin(ADDR_CO_ADC_16BIT, &Wire)) {
        Serial.println("{\"status\":\"warn\",\"msg\":\"ADS1115 16-bit ADC not detected on I2C\"}");
    }

    // 3. Initialize UART for PM2.5 sensor (Sensirion SPS30)
    PM25_SERIAL_PORT.begin(PM25_UART_BAUD, SERIAL_8N1, PIN_PM25_UART_RX, PIN_PM25_UART_TX);
    sensirion_uart_open();
    if (sps30_probe() == 0) {
        sps30_start_measurement();
    } else {
        Serial.println("{\"status\":\"warn\",\"msg\":\"PM2.5 sensor not responding on UART\"}");
    }

    // 4. Initialize SPI bus for Thermocouple Daughter Board & SD Card
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    // Ensure all SPI CS lines start HIGH (deselected) before initializing devices
    pinMode(PIN_TC_FIREBOX_CS, OUTPUT);
    digitalWrite(PIN_TC_FIREBOX_CS, HIGH);
    pinMode(PIN_TC_WATER_CS, OUTPUT);
    digitalWrite(PIN_TC_WATER_CS, HIGH);
    pinMode(PIN_SD_CARD_CS, OUTPUT);
    digitalWrite(PIN_SD_CARD_CS, HIGH);

    tcFirebox.begin();
    tcWater.begin();

    if (SD.begin(PIN_SD_CARD_CS)) {
        sdAvailable = true;
    } else {
        Serial.println("{\"status\":\"warn\",\"msg\":\"SD card mount failed; streaming USB only\"}");
    }
}

// ============================================================================
// 5. ARDUINO SETUP & MAIN 2 HZ STATE MACHINE LOOP
// ============================================================================

void setup() {
    initPeripherals();
    lastSampleTickMs = millis();
    currentState = STATE_IDLE_WAIT;
}

void loop() {
    switch (currentState) {

        // --------------------------------------------------------------------
        // STATE 0: Pace the loop to exactly 2 Hz (every 500 ms)
        // --------------------------------------------------------------------
        case STATE_IDLE_WAIT: {
            unsigned long now = millis();
            if (now - lastSampleTickMs >= SAMPLE_PERIOD_MS) {
                lastSampleTickMs += SAMPLE_PERIOD_MS;
                currentState = STATE_OPEN_JSON_RECORD;
            }
            break;
        }

        // --------------------------------------------------------------------
        // STATE 1: Begin a new JSONL line with the system timestamp
        // --------------------------------------------------------------------
        case STATE_OPEN_JSON_RECORD: {
            usbBufferIdx = 0;
            usbJsonBuffer[0] = '\0';

            if (sdAvailable) {
                jsonlFile = SD.open(JSONL_FILE_PATH, FILE_APPEND);
            }

            char header[48];
            snprintf(header, sizeof(header), "{\"ts_ms\":%lu", lastSampleTickMs);
            writeJsonChunk(header);

            currentState = STATE_READ_SAVE_TEMP_HUM;
            break;
        }

        // --------------------------------------------------------------------
        // STATE 2: Read Adafruit Temp/Humidity over I2C -> Save to JSONL
        // --------------------------------------------------------------------
        case STATE_READ_SAVE_TEMP_HUM: {
            float ambientTempC = sht31.readTemperature();
            float relativeHumPct = sht31.readHumidity();

            bool tempValid = !isnan(ambientTempC);
            bool humValid  = !isnan(relativeHumPct);

            saveJsonFloatField("amb_temp_c", ambientTempC, 2, tempValid);
            saveJsonFloatField("amb_rh_pct", relativeHumPct, 2, humValid);

            currentState = STATE_READ_SAVE_CO_ADC;
            break;
        }

        // --------------------------------------------------------------------
        // STATE 3: Read Analog CO Sensor via 16-bit I2C ADC -> Save to JSONL
        // --------------------------------------------------------------------
        case STATE_READ_SAVE_CO_ADC: {
            // Read raw 16-bit signed count and convert to millivolts
            int16_t rawCounts = ads1115.readADC_SingleEnded(CO_ADC_CHANNEL);
            float voltageMv   = ads1115.computeVolts(rawCounts) * 1000.0f;

            // Basic sanity check: disconnected/shorted rail bounds
            bool adcValid = (rawCounts >= -10 && rawCounts <= 32767);

            saveJsonIntField("co_raw", rawCounts, adcValid);
            saveJsonFloatField("co_mv", voltageMv, 3, adcValid);

            currentState = STATE_READ_SAVE_PM25;
            break;
        }

        // --------------------------------------------------------------------
        // STATE 4: Read PM2.5 Sensor over UART -> Save to JSONL
        // --------------------------------------------------------------------
        case STATE_READ_SAVE_PM25: {
            struct sps30_measurement pmData;
            uint16_t dataReady = 0;
            bool pmValid = false;

            // Poll UART sensor for new particulate concentration frame
            if (sps30_read_data_ready(&dataReady) == 0 && dataReady) {
                if (sps30_read_measurement(&pmData) == 0) {
                    pmValid = true;
                }
            }

            saveJsonFloatField("pm25_ugm3", pmData.mc_2p5, 2, pmValid);
            saveJsonFloatField("pm10_ugm3", pmData.mc_10p0, 2, pmValid);

            currentState = STATE_READ_SAVE_THERMOCOUPLES;
            break;
        }

        // --------------------------------------------------------------------
        // STATE 5: Read Firebox & Water Thermocouples -> Save to JSONL
        // --------------------------------------------------------------------
        case STATE_READ_SAVE_THERMOCOUPLES: {
            double fireboxTempC = tcFirebox.readCelsius();
            double waterTempC   = tcWater.readCelsius();

            // MAX31855 returns NAN if thermocouple is open, shorted to GND, or VCC
            bool fireboxValid = !isnan(fireboxTempC);
            bool waterValid   = !isnan(waterTempC);

            saveJsonFloatField("tc_firebox_c", (float)fireboxTempC, 2, fireboxValid);
            saveJsonFloatField("tc_water_c",   (float)waterTempC,   2, waterValid);

            currentState = STATE_FINALIZE_RECORD;
            break;
        }

        // --------------------------------------------------------------------
        // STATE 6: Close JSON object, flush SD file, and send over USB Serial
        // --------------------------------------------------------------------
        case STATE_FINALIZE_RECORD: {
            writeJsonChunk("}\n");

            if (sdAvailable && jsonlFile) {
                jsonlFile.flush();
                jsonlFile.close();
            }

            // Output the completed JSONL line over Serial USB for the Python script
            Serial.print(usbJsonBuffer);

            currentState = STATE_IDLE_WAIT;
            break;
        }
    }
}
/*
Valid Single-Line JSONL Output: Each 2 Hz pass produces one complete, self-contained JSON line 
(e.g., {"ts_ms":500,"amb_temp_c":23.41,"amb_rh_pct":41.20,"co_raw":4120,"co_mv":515.000,"pm25_ugm3":14.82,"pm10_ugm3":18.10,"tc_firebox_c":412.50,"tc_water_c":68.75}), 
writing each sensor's fields to the file in its dedicated state before advancing.

Fault-Tolerant Null Handling: If a thermocouple disconnects or a bus read fails, 
saveJsonFloatField / saveJsonIntField writes null rather than NaN, 
keeping the .jsonl file and downstream Python JSON parser from breaking mid-test.

Easy Library Swapping: If your Adafruit Temp/Humidity board uses 
an SHT40/SHT41 (Adafruit_SHT4x.h), 
a MAX6675 daughter board, 
or a PMS5003 UART sensor, 
only the driver calls inside states 2, 4, or 5 need to change while the state machine and JSONL writer stay intact.
*/