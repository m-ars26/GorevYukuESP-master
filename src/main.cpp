#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include "FS.h"
#include <SD.h>
#include <Arduino.h>
#include <NMEAParser.h>

#define R1 100
#define R2 10
#define VOLTAGE_MAX 8400
#define VOLTAGE_MIN 6600
#define VOLTAGE_OUT(Vin) (((Vin)*R2) / (R1 + R2))
#define ADC_REFERENCE 1100
#define VOLTAGE_TO_ADC(in) ((ADC_REFERENCE * (in)) / 4096)
#define BATTERY_MAX_ADC VOLTAGE_OUT(VOLTAGE_MAX)
#define BATTERY_MIN_ADC VOLTAGE_OUT(VOLTAGE_MIN)
#define BAT_ADC_EN 33
#define BAT_ADC 34
#define BATTERY_VOLTAGE_CYCLES 5

#define SD_MISO     19
#define SD_MOSI     23
#define SD_SCLK     18
#define SD_CS       5
SPIClass sdSPI(VSPI);
String dataMessage;

Adafruit_BME280 bme;
NMEAParser<2> parser;

float previousAltitude = 0.0;
uint32_t start, stop, count;
int kosul = 0, drogueDeployed = 0, mainDeployed = 0;
int drogueHoldCounter = 15;
int mainHoldCounter = 15;
//int batteryVoltageCycles = BATTERY_VOLTAGE_CYCLES;

// gönderilen data paketi
struct
{
  String timeStamp = "0";
  String pressure = "0";
  String temperature = "0";
  String gpsAltitude = "0";
  String gpsLatitude = "0";
  String gpsLongtitude = "0";
  String humidity = "0";
  //String batteryVoltage = "0";
} dataPackage;

/* kalman filtre tanımlamaları */
struct
{
  float pressureEstimate = 0;
  float pressureErrorEstimate = 0;
  float kalmanGain = 0;
  float processNoiseCovariance = 0.001;    // Q
  float measurementNoiseCovariance = 0.01; // R
} kalmanPressure;

struct
{
  float temperatureEstimate = 0;
  float temperatureErrorEstimate = 0;
  float kalmanGain = 0;
  float processNoiseCovariance = 0.001;    // Q
  float measurementNoiseCovariance = 0.01; // R
} kalmanTemperature;

struct
{
  float humidityEstimate = 0;
  float humidityErrorEstimate = 0;
  float kalmanGain = 0;
  float processNoiseCovariance = 0.008;    // Q
  float measurementNoiseCovariance = 0.05; // R
} kalmanHumidity;
/* kalman end */

void parseGpsData();
void parserErrorHandler();
void unknownNMEA();
double ddmmToDecimalDeg(double ddmm);
String utcToLocalTime(float utc_timestamp, int8_t timezone);
float calculateAltitude(float pressure, float temperature);
void sendDataToGroundStation();
float updatePressureKalmanFilter(float pressureMeasurement);
float updateTemperatureKalmanFilter(float temperatureMeasurement);
float updateHumidityKalmanFilter(float humidityMeasurement);
int calculateBatteryVoltage(int adc);
//void initSDCard();
void appendFile(fs::FS &fs, const char * path, const char * message);
void writeFile(fs::FS &fs, const char * path, const char * message);

void setup()
{
  /*Serial Initialization*/
  Serial.begin(9600);                       // LoRa and USB
  Serial2.begin(57600, SERIAL_8N1, 16, 17); // GPS
  while (!Serial);

  bool status;
  status = bme.begin(0x76); // I2C adresi 0x76 olarak ayarlanıyor. Daha yaygın olan adres 0x77'dir.
  if (!status)
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }

  /* NMEA configuration START */
  // parser.setErrorHandler(parserErrorHandler);
  parser.addHandler("GNGGA", parseGpsData);
  // parser.setDefaultHandler(unknownNMEA);
  /* NMEA configuration END */

  sdSPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  if(!SD.begin(SD_CS, sdSPI)) {
    Serial.println("Card Mount Failed");
    return;
  }
  Serial.println("1");
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("ERROR - SD card initialization failed!");
    return;    // init failed
  }
  Serial.println("2");
  // If the data.txt file doesn't exist
  // Create a file on the SD card and write the data labels
  File file = SD.open("/data1.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/data1.txt", "Reading \r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();

}


void loop()
{
  float altitude1;

  altitude1 = 44330 * (1 - pow((bme.readPressure() / 100.0F) / 1013.25, 0.190284));
  //delay(10);

  float pressure = bme.readPressure() / 100.0F;
  dataPackage.pressure = String(pressure, 2);

  float temperature = bme.readTemperature();
  dataPackage.temperature = String(temperature, 2);

  float humidity= bme.readHumidity();
  dataPackage.humidity = String(humidity, 2);

  // kalman
  float filteredPressure = updatePressureKalmanFilter(pressure);
  dataPackage.pressure = String(filteredPressure, 2);
  float filteredTemperature = updateTemperatureKalmanFilter(temperature);
  dataPackage.temperature = String(filteredTemperature, 2);
  float filteredHumidity = updateHumidityKalmanFilter(humidity);
  dataPackage.humidity = String(filteredHumidity, 2);
  // kalman end

  // if (batteryVoltageCycles == 0)
  // {
  //   digitalWrite(BAT_ADC_EN, HIGH);
  //   //delay(10);
  //   int adcValue = analogRead(BAT_ADC);
  //   //delay(10);
  //   digitalWrite(BAT_ADC_EN, LOW);
  //   int batteryVoltage = calculateBatteryVoltage(adcValue);
  //   dataPackage.batteryVoltage = String(batteryVoltage);
  //   batteryVoltageCycles = BATTERY_VOLTAGE_CYCLES;
  // }
  // else
  // {
  //   batteryVoltageCycles--;
  //   delay(10);
  // }

  while (Serial2.available()) // send gps nmea sentences to the parser
  {
    parser << Serial2.read();
  }

  sendDataToGroundStation();
  delay(100);
  //delay(10);
  if (SD.begin()){

    String timeStr = dataPackage.timeStamp;
    appendFile(SD, "/data1.txt", "Time = ");
    appendFile(SD, "/data1.txt", timeStr.c_str());
    appendFile(SD, "/data1.txt", "\t");

    String temperatureStr = dataPackage.temperature;
    appendFile(SD, "/data1.txt", "Temperature = ");
    appendFile(SD, "/data1.txt", temperatureStr.c_str());
    appendFile(SD, "/data1.txt", "\t");
  
    String pressureStr = dataPackage.pressure;
    appendFile(SD, "/data1.txt", "Pressure = ");
    appendFile(SD, "/data1.txt", pressureStr.c_str());
    appendFile(SD, "/data1.txt", "\t");
  
    String humidityStr = dataPackage.humidity ;
    appendFile(SD, "/data1.txt", "Humidity = ");
    appendFile(SD, "/data1.txt", humidityStr.c_str());
    appendFile(SD, "/data1.txt", "\n");
  
    String latStr = dataPackage.gpsLatitude;
    appendFile(SD, "/data1.txt", "Latitude = ");
    appendFile(SD, "/data1.txt", latStr.c_str());
    appendFile(SD, "/data1.txt", "\t");
    
    String lngStr = dataPackage.gpsLongtitude;
    appendFile(SD, "/data1.txt", "Longitude = ");
    appendFile(SD, "/data1.txt", lngStr.c_str());
    appendFile(SD, "/data1.txt", "\n");
    delay(100);

  }
    
}

void parseGpsData()
{
  float latitude, longtitude, altitude, timestamp;
  if (parser.getArg(0, timestamp))
  {
    dataPackage.timeStamp = utcToLocalTime(timestamp, 3);
  }
  if (parser.getArg(1, latitude))
  {
    dataPackage.gpsLatitude = String(ddmmToDecimalDeg(latitude), 6);
  }
  if (parser.getArg(3, longtitude))
  {
    dataPackage.gpsLongtitude = String(ddmmToDecimalDeg(longtitude), 6);
  }
  if (parser.getArg(8, altitude))
  {
    dataPackage.gpsAltitude = String(altitude, 3);
  }
}

void parserErrorHandler()
{
  Serial.print("*** ERROR: ");
  Serial.println(parser.error());
}

void unknownNMEA()
{
  Serial.print("*** Unkown command: ");
  char buf[6];
  parser.getType(buf);
  Serial.println(buf);
}

double ddmmToDecimalDeg(double ddmm)
{
  int degrees = int(ddmm / 100);
  double minutes = ddmm - (degrees * 100);
  return degrees + (minutes / 60);
}

String utcToLocalTime(float utc_timestamp, int8_t timezone)
{
  int utc_hours = int(utc_timestamp / 10000);
  int utc_minutes = int((utc_timestamp - (utc_hours * 10000)) / 100);
  float utc_seconds = utc_timestamp - (utc_hours * 10000) - (utc_minutes * 100);

  int local_hours = utc_hours + timezone;
  if (local_hours >= 24)
  {
    local_hours -= 24;
  }
  else if (local_hours < 0)
  {
    local_hours += 24;
  }

  String local_time = "";
  if (local_hours < 10)
    local_time += "0";
  local_time += String(local_hours) + ":";
  if (utc_minutes < 10)
    local_time += "0";
  local_time += String(utc_minutes) + ":";
  if (utc_seconds < 10)
    local_time += "0";
  local_time += String(utc_seconds, 3);

  return local_time;
}

float calculateAltitude(float pressure, float temperature)
{
  const float sea_level_pressure = 1013.25; // sea level standard atmospheric pressure in hPa
  const float R = 287.05;                   // specific gas constant for dry air in J/(kg·K)
  const float g0 = 9.80665;                 // standard acceleration of gravity in m/s²
  const float T0 = 273.15;                  // standard temperature at sea level in Kelvin

  float altitude = ((powf((sea_level_pressure / pressure), (1 / 5.257)) - 1) * (temperature + T0)) / 0.0065;
  return altitude;
}

void sendDataToGroundStation()
{
  Serial.print(dataPackage.timeStamp);
  Serial.print(",");
  Serial.print(dataPackage.pressure);
  Serial.print(",");
  Serial.print(dataPackage.temperature);
  Serial.print(",");
  Serial.print(dataPackage.humidity);
  Serial.print(",");
  Serial.print(dataPackage.gpsLatitude);
  Serial.print(",");
  Serial.print(dataPackage.gpsLongtitude);
  Serial.print(",");
  Serial.print(dataPackage.gpsAltitude);
  //Serial.print(",");
  //Serial.print(dataPackage.batteryVoltage);
  Serial.print("\n");
}

float updatePressureKalmanFilter(float pressureMeasurement)
{
  // prediction
  float pressureErrorPrediction = kalmanPressure.pressureErrorEstimate + kalmanPressure.processNoiseCovariance;

  // update
  kalmanPressure.kalmanGain = pressureErrorPrediction / (pressureErrorPrediction + kalmanPressure.measurementNoiseCovariance);
  kalmanPressure.pressureEstimate = kalmanPressure.pressureEstimate + kalmanPressure.kalmanGain * (pressureMeasurement - kalmanPressure.pressureEstimate);
  kalmanPressure.pressureErrorEstimate = (1 - kalmanPressure.kalmanGain) * pressureErrorPrediction;

  return kalmanPressure.pressureEstimate;
}

float updateTemperatureKalmanFilter(float temperatureMeasurement)
{
  // prediction
  float temperatureErrorPrediction = kalmanTemperature.temperatureErrorEstimate + kalmanTemperature.processNoiseCovariance;

  // update
  kalmanTemperature.kalmanGain = temperatureErrorPrediction / (temperatureErrorPrediction + kalmanTemperature.measurementNoiseCovariance);
  kalmanTemperature.temperatureEstimate = kalmanTemperature.temperatureEstimate + kalmanTemperature.kalmanGain * (temperatureMeasurement - kalmanTemperature.temperatureEstimate);
  kalmanTemperature.temperatureErrorEstimate = (1 - kalmanTemperature.kalmanGain) * temperatureErrorPrediction;

  return kalmanTemperature.temperatureEstimate;
}

float updateHumidityKalmanFilter(float humidityMeasurement)
{
  // prediction
  float humidityErrorPrediction = kalmanHumidity.humidityErrorEstimate + kalmanHumidity.processNoiseCovariance;

  // update
  kalmanHumidity.kalmanGain = humidityErrorPrediction / (humidityErrorPrediction + kalmanHumidity.measurementNoiseCovariance);
  kalmanHumidity.humidityEstimate = kalmanHumidity.humidityEstimate + kalmanHumidity.kalmanGain * (humidityMeasurement - kalmanHumidity.humidityEstimate);
  kalmanHumidity.humidityErrorEstimate = (1 - kalmanHumidity.kalmanGain) * humidityErrorPrediction;

  return kalmanHumidity.humidityEstimate;
}

// int calculateBatteryVoltage(int adc)
// {
//   int battery = adc * (R1 + R2) / R2;
//   return battery;
// }

/*void initSDCard(){
    if (!SD.begin()) {
        Serial.println("SD card initialization failed!");
        return;
    }
    Serial.println("SD card initialized successfully");
}
*/

  void writeFile(fs::FS &fs, const char * path, const char * message) {
  

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    return;
  }
  if(file.print(message)) {
    
  } else {
    
  }
  file.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    
    return;
  }
  if(file.print(message)) {
    
  } else {
    
  }
  file.close();
}