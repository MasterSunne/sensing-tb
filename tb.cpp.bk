#include <ArduinoJson.h>
#include <SD.h>
#include <SPI.h>

const int chipSelect = 53; // Mega's hardware SS
File dataFile;

void setup() {
  SD.begin(chipSelect);
  dataFile = SD.open("data.jsonl", FILE_WRITE);
}

void loop() {
  // Read your sensors
  long ts = millis();
  float temp = 72.5;
  float hum  = 45.0;

  // Build one tiny record (~150 bytes RAM)
  JsonDocument doc;
  doc["ts"]   = ts;
  doc["temp"] = temp;
  doc["hum"]  = hum;

  // Write one line to SD, then it's gone
  serializeJson(doc, dataFile);
  dataFile.println();  // newline = JSONL

  // doc is freed here (scope ends or next iteration)
  delay(1000);
}   