/*******************************************************************************
* AmbaSat-1 Low Earth Orbit Space Satellite
* Filename: main.h
*
* Copyright (c) 2021 PY FOUNDTION GROUP SA
* http://pyfoundation.org
*
* licensed under Creative Commons Attribution-ShareAlike 3.0
* ******************************************************************************
*
* Supporting code/libraries:
*
* LoRa code Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
*
* Permission is hereby granted, free of charge, to anyone
* obtaining a copy of this document and accompanying files,
* to do whatever they want with them without any restriction,
* including, but not limited to, copying, modification and redistribution.
* NO WARRANTY OF ANY KIND IS PROVIDED.
**
*******************************************************************************/

#include <Arduino.h>
#include <Debugging.h>
#include "LowPower.h"

//####################  BME680 ####################
//####################  BME680 ####################
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

//####################  BME680 ####################
//####################  BME680 ####################

//####################  TTN ####################
//####################  TTN ####################
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"
#include "i2c.h"
#include <Arduino.h>
#include <LoraMessage.h>
#include "settings.h"

#define addr 0x4A
#define LedPin 9
#define ADR_MODE 1

//bool joined = false;
//bool sleeping = false;

// -----------------------------------------------------------------------------
// Gyro/Magno/Accel structure
// -----------------------------------------------------------------------------
/*
struct threeDData 
{
  	uint8_t x;
	uint8_t y;
	uint8_t z;
};
*/
// -----------------------------------------------------------------------------
// TTN payload data structure - see https://www.thethingsnetwork.org/docs/devices/bytes.html
// -----------------------------------------------------------------------------
/*
struct
{
  uint8_t sensorType;
	uint8_t field1; 
  float field2; 
	uint16_t voltage;
} packetData;
*/
// TTN *****************************  IMPORTANT 
// 
// Set the following three values to match your unique AmbaSat-1 satellite   
// 
// The Network Session Key
static const PROGMEM u1_t NWKSKEY[16] = {0x03,0x80,0xD8,0x0E,0x70,0x52,0x0E,0xB0,0x4F,0x08,0x10,0x0F,0x0D,0x90,0xA5,0x5A}; //<< CHANGE

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = {0x06,0xB0,0x98,0x0F,0x30,0x21,0x04,0x60,0xBF,0x03,0x50,0x20,0x02,0x10,0x5A,0xA5}; //<< CHANGE

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26021CD4;  //<< CHANGE

/*
const char *devAddr = "26021CD4";
const char *nwkSKey = "0380D80E70520EB04F08100F0D90A55A";
const char *appSKey = "06B0980F30210460BF03502002105AA5";
Application ID
emegency0001
Device ID
cjcontainner0001
Description
PYFoundationSat-1
*/
/***********************************  IMPORTANT */


int sleepcycles = 13; // 130 X 8 seconds = ~17 mins sleep


// These callbacks are only used in over-the-air activation, so they are
// left empty here (cannot be left out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
/*
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
*/
/*
static osjob_t sendjob;
static osjob_t initjob;
*/
/*mapping
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 7,
  .dio = {2, A2, LMIC_UNUSED_PIN},
};
*/
//####################  TTN ####################
//####################  TTN ####################

//####################  BME680 ####################
//####################  BME680 ####################

#define SEALEVELPRESSURE_HPA (1013.25)
#define BME680_DEFAULT_ADDRESS_PYFOUNDATIONSAT  (0x76)     ///< AmbaSat BME680 address

//####################  BME680 ####################
//####################  BME680 ####################

#define addr 0x4A
#define SERIAL_BAUD         9600 
#define LED_PIN             9

// AmbaSat-1 range of I2C compatible sensors and their TTN port IDs / Dashboard - config.inc
#define SENSOR_01_SHT31     51 // Temperature & Humidity
#define SENSOR_02_STS21     52 // Temperature
#define SENSOR_03_BME680    53 // Gas, Pressure, TVOC 
#define SENSOR_04_OPT3001   54 // Ambient Light
#define SENSOR_05_ZMOD4410  55 // TVOC 
#define SENSOR_06_SI1132    56 // UV 7 Ambient
#define SENSOR_07_CCS811B   57 // TVO & CO2
#define SENSOR_08_TESEO     58 // GPS

// LSM9DS1 Addresses
#define LSM9DS1_M	0x1E  // Mag address
#define LSM9DS1_AG	0x6B  // AG address


// Set the sensor type 
static const uint8_t sensorType = SENSOR_03_BME680; // for test purposes


uint16_t readVcc();

// -----------------------------------------------------------------------------
// TTN payloads - see: https://www.thethingsnetwork.org/docs/devices/bytes.html
// -----------------------------------------------------------------------------

// TTN decode for SHT31 is as below
//  
// 'temperature' uses a 16bit two's complement with two decimals, so the range is -327.68 to +327.67 degrees
// 'humidity' is rounded to whole percent. eg 55.5% becomes 55%
// 'voltage' - eg. 2840 millivolts becomes 28 and in the Dashboard is decoded to 2.8 volts
//
// AmbaSat 'SENSOR_01_SHT31' TTN Payload Decoder expects:
// decode(bytes, [temperature, uint8, uint8, int16, int16, int16, int16, int16, int16, int16, int16, int16], ['temperature', 'humidity', 'voltage', 'gx', 'gy', 'gz', 'ax', 'ay', 'az', 'mx', 'my', 'mz']);
