/*
 * Connect to Wifi and set time
 * 
 * display time and rssi on SSD1306 I2C display
 * 
 * Read temperature from DS18B20 via w1
 * Read temperature from MAX6675 via SPI
 * 
 * Write data back when wlan available
 * 
 * Scanning...
 * I2C device found at address 0x3C - display
 * I2C device found at address 0x68 - gyrometer MPU6050
 * I2C device found at address 0x76 - bme280
 * 
 * 
 * 
 * 
 * 
 * ToDo: 
 * Break out sensor reading in own thread
 * Write multiple/batched
 * bidirectional communication / slack?
 */

#define DEVICE "ESP32"

#include <WiFi.h>
#include "time.h"
#include "SSD1306Wire.h"        
#include <TFT_eSPI.h>           // Hardware-specific library
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <LinkedList.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include "credentials.h"

// should be adjustable with a potentiometer
const int   loopWait = 1000;

static const int spiClk = 1000000; // 1 MHz

struct Measurement {
  int rssi;
  unsigned int freeheap;
  unsigned int listlength;
  float w1_tempC;
  float spi_tempC;
  float rpmfreq;
  unsigned int rpm;
  float bme280_airpressure;
  float bme280_humidity;
  float bme280_temperature;
  float bme280_altitude;
  float mpu6050_accel_x;
  float mpu6050_accel_y;
  float mpu6050_accel_z;
  float mpu6050_gyro_x;
  float mpu6050_gyro_y;
  float mpu6050_gyro_z;  
  float mpu6050_temperature;
  unsigned int analog0;
  // We assume 0-5 volt
  float afrvoltage0;
  unsigned int sensorreadtime;
  unsigned int lastwritetime;
  wl_status_t wlanstatus;
  time_t timestamp;
  // debugging
  unsigned int readtime_w1_temp;
  unsigned int readtime_bme280;
  unsigned int readtime_mpu6050;
  unsigned int readtime_max6675;
  unsigned int readtime_mcp3008;
  unsigned int readtime_esp32;
  unsigned int readtime_interrupts;
  unsigned int writetime_display;
};

unsigned long rpmpulses=0;
unsigned long oldrpmpulses=0;
unsigned int curRPM=0;

Adafruit_BME280 bme; // I2C
unsigned bme280status;

// MPU6050
Adafruit_MPU6050 mpu;

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;

// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

char timestring[50];
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;

#ifdef HAVE_SSD1306
// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h
#endif
#ifdef HAVE_ST7735
TFT_eSPI tft = TFT_eSPI();       // Invoke custom library
#endif

// Create the influxdb object
InfluxDBClient influxclient(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// This is where we save data
LinkedList<Measurement> myMeasurements = LinkedList<Measurement>();
Measurement M;
Measurement curM;

// amateurish way to figure out if we have an ntp sync
# define FEBRUARY2020 1581627570
unsigned int timeiteration = 0;
time_t setuptime;

// to catch quick errors with the influx connection
bool hadinfluxerrors = 0;
Point row(INFLUXDB_ROWNAME);

void IRAM_ATTR interruptserviceroutine() {
  rpmpulses++;
}
unsigned long int rpmmillis;
unsigned long int lastwritetime=0;
unsigned long int startwrite;

void setup() {
    Serial.begin(115200);
    delay(1);
    time(&setuptime);
    // Initialising the UI will init the display too.
    Serial.print(String(setuptime));
    Serial.println("Initializing display");
#ifdef HAVE_SSD1306
    display.init();
    display.flipScreenVertically();
#endif
#ifdef HAVE_ST7735
    tft.init();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(0, 0);
#endif
    printStringOnDisplay("Connecting to Wifi");
    // connect to the wifi network

    Serial.println();
    time(&setuptime);
    Serial.print(String(setuptime));
    Serial.print(" Connecting to ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    printStringOnDisplay("Wifi connected");
    Serial.println("");
    time(&setuptime);
    Serial.print(String(setuptime));
    Serial.println(" WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("ESP Mac Address: ");
    Serial.println(WiFi.macAddress());

    // there is no point in writing data unless we have a proper time.
    // halt until good time
    printStringOnDisplay("Setting time ");
    configTime(gmtOffset_sec, daylightOffset_sec, NTPSERVER);
    time(&setuptime);
    while (setuptime < FEBRUARY2020) {
      displayWhileWaitingForTime();
      timeiteration++;
      Serial.println("Time attempt " + String(timeiteration) + " " + String(setuptime));
      configTime(gmtOffset_sec, daylightOffset_sec, NTPSERVER);
      time(&setuptime);
      delay(1000);
    }    
    //configure the influxdb connection
    printStringOnDisplay("connect to influx, ");
    influxclient.setWriteOptions(INFLUXDB_WRITE_PRECISION);

#ifdef HAVE_W1
    // initialize w1 bus
    printStringOnDisplay("init w1 bus, ");
    sensors.begin();
#endif
#ifdef HAVE_BME280
    printStringOnDisplay("init BME280 i2c, 0x76, ");
    // default address for the BME280 is in this library 0x60
    bme280status = bme.begin(0x76);
    if (!bme280status) {
      printStringOnDisplay("BME280 failed, ");
    }
#endif
#ifdef HAVE_MPU6050
    // initialize MPU6050
    printStringOnDisplay("init MPU6050 i2c, 0x68");
    if (!mpu.begin()) {
      printStringOnDisplay("MPU6050 failed");
    }
    MPU6050calibration();
#endif
    //initialise instance of the SPIClass attached to VSPI (perhaps we need HSPI later)
    printStringOnDisplay("init SPI bus, ");
    vspi = new SPIClass(VSPI);
    //initialise vspi with default pins
    //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
    vspi->begin();
    //set up slave select pins as outputs as the Arduino API
    //doesn't handle automatically pulling SS low
    pinMode(MAX6675_CS, OUTPUT); //VSPI SS for first SPI device (MAX6675)
//    pinMode(2, OUTPUT); //VSPI SS for second SPI device

    // create interrupt routine for GPIO4
    attachInterrupt(4, interruptserviceroutine, RISING);
    
    printStringOnDisplay("setup finished");
    rpmmillis=millis();
    oldrpmpulses=rpmpulses;
}

void loop() {
  readAllData(&M);
  myMeasurements.add(M);
//  printAllInfo(&M);
  printOnDisplay(&M);
  if (M.wlanstatus != WL_CONNECTED && M.w1_tempC != DEVICE_DISCONNECTED_C) {
    Serial.print("Wifi or w1 sensor not connected, WiFi.status = ");
    Serial.println(wl_status_to_string(M.wlanstatus));
  }
  else 
  {
    // We have connectivity, sync time
    configTime(gmtOffset_sec, daylightOffset_sec, NTPSERVER);
    // pop all measurements and write them to the db, verify that we have connectivity while we do it
    // since we pop first we lose a measurement if the wifi is lost while emptying the list
    hadinfluxerrors = 0;
    while (WiFi.status() == WL_CONNECTED && myMeasurements.size() > 0 && hadinfluxerrors == 0) {
      // curM is pulled from the list
      curM = myMeasurements.pop();
      // Write to the DB
      row.clearFields();
      row.clearTags();
      row.addField("rssi", curM.rssi);
      row.addField("freeheap", curM.freeheap);
      row.addField("listlength", curM.listlength);
      row.addField("wlstatus", curM.wlanstatus);
#ifdef HAVE_BME280
      row.addField("bme280_temperature",curM.bme280_temperature);
      row.addField("bme280_humidity",curM.bme280_humidity);
      row.addField("bme280_airpressure",curM.bme280_airpressure);
      row.addField("readtime_bme280",curM.readtime_bme280);
#endif
#ifdef HAVE_MPU6050
      row.addField("mpu6050_accel_x",curM.mpu6050_accel_x);
      row.addField("mpu6050_accel_y",curM.mpu6050_accel_y);
      row.addField("mpu6050_accel_z",curM.mpu6050_accel_z);
      row.addField("mpu6050_gyro_x",curM.mpu6050_gyro_x);
      row.addField("mpu6050_gyro_y",curM.mpu6050_gyro_y);
      row.addField("mpu6050_gyro_z",curM.mpu6050_gyro_z);
      row.addField("mpu6050_temperature",curM.mpu6050_temperature);
      row.addField("readtime_mpu6050",curM.readtime_mpu6050);
#endif
#ifdef HAVE_W1
      row.addField("w1_tempC", curM.w1_tempC);
      row.addField("readtime_w1_temp",curM.readtime_w1_temp);
#endif
#ifdef HAVE_MAX6675
      row.addField("max6675_tempC", curM.spi_tempC);
      row.addField("readtime_max6675",curM.readtime_max6675);
#endif
#ifdef HAVE_MCP3008
      row.addField("readtime_mcp3008",curM.readtime_mcp3008);
      row.addField("afrvoltage0",curM.afrvoltage0);
#endif
      row.addField("readtime_esp32",curM.readtime_esp32);
      row.addField("sensorreadtime",curM.sensorreadtime);
      row.addField("lastwritetime",curM.lastwritetime);
#ifdef HAVE_INTERRUPTS
      row.addField("rpm", curM.rpm);
      row.addField("rpmfreq", curM.rpmfreq);
      row.addField("readtime_interrupts",curM.readtime_interrupts);
#endif
      row.addTag("device", "ESP32");
      row.setTime(curM.timestamp);
      printAllInfo(&curM);
// we get a guru meditation of this fails more than 10-15 times
      startwrite=millis();
#ifdef REALLY_UPLOAD
      if(!influxclient.writePoint(row)) {
        Serial.println("Influx Error: " + influxclient.getLastErrorMessage());
        // We failed for some reason, put the Measurement back in the list
        myMeasurements.add(curM);
        hadinfluxerrors = 1;
      }
#endif
      lastwritetime=millis() - startwrite;
    }
  }
  delay (loopWait);
}
