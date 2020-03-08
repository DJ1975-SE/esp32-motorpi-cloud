#define WIFI_SSID "yourssid"
#define WIFI_PASSWORD "yourpassword"
#define NTPSERVER "0.de.pool.ntp.org"

// for cloud and local
#define INFLUXDB_URL "the url from influxdatas cloud"
#define INFLUXDB_WRITE_PRECISION WritePrecision::S

// for local v1
#define INFLUXDB_USER "influxdb user"
#define INFLUXDB_PASS "influxdb password"
#define INFLUXDB_DATABASE "influxdb database"

// for cloud
#define INFLUXDB_ORG "generally cloud login, i.e. email address"
#define INFLUXDB_TOKEN "generated from webUI"
#define INFLUXDB_ROWNAME "row name"
#define INFLUXDB_BUCKET "bucket, generated from webUI"

// Fingerprint of Certificate Authority of InfluxData Cloud 2 servers
const char InfluxDbCloud2CAFingerprint[] PROGMEM = "9B:62:0A:63:8B:B1:D2:CA:5E:DF:42:6E:A3:EE:1F:19:36:48:71:1F";

//#define REALLY_UPLOAD 1
//#define HAVE_BME280   1
//#define HAVE_MPU6050  1
//#define HAVE_W1       1
#define HAVE_MAX6675  1
#define HAVE_ST7735   1
//#define HAVE_SSD1306   1
//#define HAVE_MCP3008  1
#define HAVE_INTERRUPTS 1

//Pinout for ST7735 is in the "User_Setup.h" file in the TFT_eSPI library

//Pin for MAX6675 CS
#define MAX6675_CS 2

// Data wire is connected to GPIO15
#define ONE_WIRE_BUS 5

// BME280 Calibration
#define SEALEVELPRESSURE_HPA (1013.25)

//MCP3008 SPI AD Converter
#define MCP3008_SPI_MAX_5V 3600000         ///< SPI MAX Value on 5V pin
#define MCP3008_SPI_MAX_3V 1350000         ///< SPI MAX Value on 3V pin
#define MCP3008_SPI_MAX MCP3008_SPI_MAX_5V ///< SPI MAX Value
#define MCP3008_SPI_ORDER MSBFIRST         ///<  SPI ORDER
#define MCP3008_SPI_MODE SPI_MODE0         ///< SPI MODE
#define MCP3008_VREF 5.1
