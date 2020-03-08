# Read values with an ESP32 and store them in a cloudbased influxdb

This is code in development which reads various sensors connected to an ESP32 and stores them locally, and whenever there is Wifi connectivity, it dumps it off in an influxdb.

The code is poor in the following ways:

* no peer review
* mostly assuming sensors are always OK
* superhappy fun time with global variables, break statements etc
* designed during creation
* possibly buggy, no real testing yet

## Data Storage

* Influxdatas https://www.influxdata.com/ cloud offer

## To-do list

* Split code on different cores, perhaps breaking out the sensor reading on the free core
* Fix issue when recovery from missing connectivity, until local buffer has been written away no new values are being read
* A ton of other things
* List the dependencies' repos / URLs / details (Arduino libraries)
* be more efficient with RAM (datatypes in struct)

## Pinout / Wiring

(for ST7735 display: Pinout is defined in User_Setup.h)

* ESP GPIO18 = SPI CLK (MCP3008, MAX6675, ST7735)
* ESP GPIO19 = SPI Data MISO (MCP3008, MAX6675)
* ESP GPIO23 = SPI Data MOSI (MCP3008, ST7735)
* ESP GPIO2  = SPI CS for MAX6675
* ESP GPIOX  = SPI CS for MCP3008
* ESP GPIO15 = SPI CS for ST7735
* ESP GPIO4  = Data Command for ST7735
* ESP GPIO5  = Dallas 1Wire Data for DS18B20
* ESP GPIO22 = I2C CLK (MPU6050, DME280, SSD1306)
* ESP GPIO21 = I2C SDA
* +3V3 via 330Ohm R = RESET for ST7735

The MCP3008 have a +5 Vin (Vin cannot be lower than Vref), the rest is running on 3.3 V

