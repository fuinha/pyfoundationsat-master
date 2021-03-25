/*******************************************************************************
* PYfoundationSat-1 Low Earth Orbit Space Satellite
* Filename: main.cpp
* PYfoundationSat-1 Flight Code for Sensor 01 - SHT31: Temperature & Humidity
* 21st February 2021
* Authors: Eng. Marcelo Anjos
*
* Copyright (c) 2021 PY Foundation Group SA
* http://pyfoundation.org
*
* To use this code, set NWKSKEY, APPSKEY & DEVADDR values as per your Dashboard
* See the HowTo: https://ambasat.com/howto/kit-2/#/../unique-ids
*
* For ISM band configuration: See lmic/config.h eg. #define CFG_us915 1
* licensed under Creative Commons Attribution-ShareAlike 3.0
* ******************************************************************************/
/*******************************************************************************
* AmbaSat-1 TTN
* 17th January 2021
* Version 1.0
* Filename: main.cpp
*
* Copyright (c) 2020 AmbaSat Ltd
* https://ambasat.com
*
* This application will submit test data to The Things Network (TTN)
* Data is routed by TTN to the AmbaSat Dashboard
* You MUST set your unique AmbaSat-1 KEYS & DEVICE ID in main.h
*
* Licensed under Creative Commons Attribution-ShareAlike 3.0
*
* This example will send Temperature, humidity & battery information
* using frequency and encryption settings matching those of
* the The Things Network.
*
* This example uses ABP (Activation-by-personalisation), where a DevAddr and
* Session keys are preconfigured (unlike OTAA, where a DevEUI and
* application key is configured, while the DevAddr and session keys are
* assigned/generated in the over-the-air-activation procedure).
**
* Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
* g1, 0.1% in g2), but not the TTN fair usage policy.

* To use this code, first register your application and device with
* the things network, to set or generate an AppEUI, DevEUI and AppKey.
* Multiple devices can use the same AppEUI, but each device has its own
* DevEUI and AppKey.
*
* Supporting code/libraries:
*
* Based on code originally created by
* Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
*
* Permission is hereby granted, free of charge, to anyone
* obtaining a copy of this document and accompanying files,
* to do whatever they want with them without any restriction,
* including, but not limited to, copying, modification and redistribution.
* NO WARRANTY OF ANY KIND IS PROVIDED.
*
* LowPower Library
* Author: Lim Phang Moh
* Company: Rocket Scream Electronics
* Website: www.rocketscream.com
* This is a lightweight low power library for Arduino.
* The LowPower Library is licensed under Creative Commons Attribution-ShareAlike 3.0

*
* NOTE. You MUST set your unique AmbaSat-1 KEYS & DEVICE ID in main.h
*
*******************************************************************************/
#include "main.h"
//#include "AmbaSatSHT31.h"
#include "AmbaSatLMIC.h"
#include "AmbaSatLSM9DS1.h"


// AmbaSat Library objects (see: https://platformio.org/lib/search?query=ambasat)
//AmbaSatSHT31 *pyfoundationSatSHT31;
AmbaSatLMIC *pyfoundationSatLMIC;
AmbaSatLSM9DS1 *pyfoundationSatLSM9DS1;

//####################  BME680 ####################
//####################  BME680 ####################
Adafruit_BME680 bme; // I2C

//#define SEALEVELPRESSURE_HPA (1013.25)
//#define BME680_DEFAULT_ADDRESS_PYFOUNDATIONSAT  (0x76)     ///< AmbaSat BME680 address

//####################  BME680 ####################
//####################  BME680 ####################

// ============================================================================

void setup()
{
    Serial.begin(9600);

    while (!Serial)
        delay(10);    

    // Turn the LED ON 
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(LED_PIN, HIGH);

    // Create the LMIC object
    pyfoundationSatLMIC = new AmbaSatLMIC();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are provided.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    
    // Setup LMIC
    pyfoundationSatLMIC->setup(0x13, DEVADDR, appskey, nwkskey);

    // Turn the LED OFF
    digitalWrite(LED_PIN, LOW);

    // Create the sensor object
    //pyfoundationSatSHT31 = new AmbaSatSHT31();

    // Create the IMU object
    pyfoundationSatLSM9DS1 = new AmbaSatLSM9DS1();

    // Initialise IMU
    if (pyfoundationSatLSM9DS1->begin(LSM9DS1_AG, LSM9DS1_M) == false) 
    {
        PRINTLN_DEBUG(F("F LSM9DS1"));
        //while (1);
    }

    if (!bme.begin(BME680_DEFAULT_ADDRESS_PYFOUNDATIONSAT)) 
    {
        PRINTLN_DEBUG(F("F BME680 "));
        //while (1);
    }

    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms

    PRINTLN_DEBUG("Setup complete");    
}

// ============================================================================

void loop()
{
    // Turn the LED OFF
    digitalWrite(LED_PIN, HIGH);
    uint16_t voltage;
    LoraMessage message;

    voltage = readVcc();

/*
    // read SHT31
    if (pyfoundationSatSHT31->getSensorReading(&temperature, &humidity)  == false) 
    {
        PRINTLN_DEBUG(F("Failed to read sensor data"));
    }  
      
    PRINT_DEBUG(F("TEMP: "));
    PRINT_DEBUG(temperature);   
    PRINT_DEBUG(F(", HUM: "));
    PRINT_DEBUG(humidity);   
    PRINT_DEBUG(F(", VOLT: "));
    PRINTLN_DEBUG(voltage);   

    uint8_t humidityEncoded = (uint8_t) humidity;       // eg. 55.4% becomes 55% 
    */


//####################  BME680 ####################
//####################  BME680 ####################

    // Tell BME680 to begin measurement.
    /*
    unsigned long endTime = bme.beginReading();
    if (endTime == 0) {
        PRINTLN_DEBUG(F("Failed to begin reading :("));
        return;
    }
   
    PRINT_DEBUG(F("Reading started at "));
    PRINT_DEBUG(millis());
    PRINT_DEBUG(F(" and will finish at "));
    PRINTLN_DEBUG(endTime);

    PRINTLN_DEBUG(F("You can do other work during BME680 measurement."));
    delay(50); // This represents parallel work.

    // There's no need to delay() until millis() >= endTime: bme.endReading()
    // takes care of that. It's okay for parallel work to take longer than
    // BME680's measurement time.

    // Obtain measurement results from BME680. Note that this operation isn't
    // instantaneous even if milli() >= endTime due to I2C/SPI latency.
    if (!bme.endReading()) 
    {
        PRINTLN_DEBUG(F("Failed to complete reading :("));
        return;
    }

    PRINT_DEBUG(F("Reading completed at "));
    PRINTLN_DEBUG(millis());
   */
    //pressure = bme.pressure / 100.0;
    //altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
   // gas = bme.gas_resistance / 1000.0;
   // temperature = bme.temperature;
    //humidity = bme.humidity;
///*
    PRINT_DEBUG(F("PRESSURE: "));
    PRINT_DEBUG(bme.pressure / 100.0);   
    PRINT_DEBUG(F(" hPa"));
   // PRINT_DEBUG(F(", ALTITUDE: "));
    //PRINT_DEBUG(bme.readAltitude(SEALEVELPRESSURE_HPA));   
   // PRINT_DEBUG(F(" mts"));
    PRINT_DEBUG(F(", GAS: "));
    PRINTLN_DEBUG(bme.gas_resistance / 1000.0);
    PRINT_DEBUG(F(" KOhms"));
    PRINT_DEBUG(F("TEMP: "));
    PRINT_DEBUG(bme.temperature);   
    PRINT_DEBUG(F(" *C"));
    PRINT_DEBUG(F(", HUM: "));
    PRINT_DEBUG(bme.humidity);   
    PRINT_DEBUG(F(" %"));
    PRINT_DEBUG(F(", VOLT: "));
    PRINTLN_DEBUG(voltage);
    PRINT_DEBUG(F(" Volts"));
//*/
//####################  BME680 ####################
//####################  BME680 ####################

    uint8_t pressureEncoded = (uint8_t) bme.pressure / 100;       // eg. 55.4% becomes 55% 
    uint8_t humidityEncoded = (uint8_t) bme.humidity;       // eg. 55.4% becomes 55% 

    uint8_t voltsEncoded = (uint8_t) (voltage / 100);   // eg. 2840 millivolts becomes 28 and in the Dashboard decoded to 2.8 volts

    // read LSM9DS1
    pyfoundationSatLSM9DS1->readGyro();
    pyfoundationSatLSM9DS1->readAccel();
    pyfoundationSatLSM9DS1->readMag();
///*
    PRINT_DEBUG(F("GYRO: "));
    PRINT_DEBUG(pyfoundationSatLSM9DS1->gx);
    PRINT_DEBUG(F(", "));
    PRINT_DEBUG(pyfoundationSatLSM9DS1->gy);
    PRINT_DEBUG(F(", "));
    PRINTLN_DEBUG(pyfoundationSatLSM9DS1->gz);    

    PRINT_DEBUG(F("ACCEL: "));
    PRINT_DEBUG(pyfoundationSatLSM9DS1->ax);
    PRINT_DEBUG(F(", "));
    PRINT_DEBUG(pyfoundationSatLSM9DS1->ay);
    PRINT_DEBUG(F(", "));
    PRINTLN_DEBUG(pyfoundationSatLSM9DS1->az);      

    PRINT_DEBUG(F("MAG: "));
    PRINT_DEBUG(pyfoundationSatLSM9DS1->mx);
    PRINT_DEBUG(F(", "));
    PRINT_DEBUG(pyfoundationSatLSM9DS1->my);
    PRINT_DEBUG(F(", "));
    PRINTLN_DEBUG(pyfoundationSatLSM9DS1->mz);  
 //*/
   // ***********************************************
   // Prepare message payload. NOTE: The order of the
   // payload data is important for the TTN decoder. 
   // Do not change the order
   // **********************************************

    // Temperature uses a 16bit two's complement with two decimals, so the range is -327.68 to +327.67 degrees
 
    message.addUint8(pressureEncoded); 
   // message.addUint16(bme.readAltitude(SEALEVELPRESSURE_HPA)); 
    message.addUint16(bme.gas_resistance / 1000.0); 
  
    message.addTemperature(bme.temperature); 
    
    message.addUint8(humidityEncoded);
    
    message.addUint8(voltsEncoded);

    message.addUint16(pyfoundationSatLSM9DS1->gx); 
    message.addUint16(pyfoundationSatLSM9DS1->gy); 
    message.addUint16(pyfoundationSatLSM9DS1->gz); 
        
    message.addUint16(pyfoundationSatLSM9DS1->ax); 
    message.addUint16(pyfoundationSatLSM9DS1->ay); 
    message.addUint16(pyfoundationSatLSM9DS1->az); 

    message.addUint16(pyfoundationSatLSM9DS1->mx); 
    message.addUint16(pyfoundationSatLSM9DS1->my); 
    message.addUint16(pyfoundationSatLSM9DS1->mz); 

    // send payload 
    pyfoundationSatLMIC->sendSensorPayload(SENSOR_03_BME680, message);  

    // Turn the LED OFF
    digitalWrite(LED_PIN, LOW);
    // sleep 8 seconds * sleepcycles
    for (int i=0; i < sleepcycles; i++)
    {
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    
    }     
}

// ============================================================================

uint16_t readVcc()
{
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
        ADMUX = _BV(MUX5) | _BV(MUX0);
    #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
        ADMUX = _BV(MUX3) | _BV(MUX2);
    #else
        ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #endif

    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring

    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    long result = (high<<8) | low;

    result = 1125300L / result;
    return result; // Vcc in millivolts
}