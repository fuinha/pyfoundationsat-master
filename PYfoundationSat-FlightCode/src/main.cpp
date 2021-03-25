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
#include "AmbaSatSHT31.h"
#include "AmbaSatLMIC.h"
#include "AmbaSatLSM9DS1.h"


// AmbaSat Library objects (see: https://platformio.org/lib/search?query=ambasat)
AmbaSatSHT31 *pyfoundationSatSHT31;
AmbaSatLMIC *pyfoundationSatLMIC;
AmbaSatLSM9DS1 *pyfoundationSatLSM9DS1;

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
    pyfoundationSatSHT31 = new AmbaSatSHT31();

    // Create the IMU object
    pyfoundationSatLSM9DS1 = new AmbaSatLSM9DS1();

    // Initialise IMU
    if (pyfoundationSatLSM9DS1->begin(LSM9DS1_AG, LSM9DS1_M) == false) 
    {
        PRINTLN_DEBUG(F("Failed to communicate with the AmbaSat-1 LSM9DS1 IMU"));
        while (1);
    }

    PRINTLN_DEBUG("Setup complete");    
}

// ============================================================================

void loop()
{
    float temperature, humidity;
    uint16_t voltage;
    LoraMessage message;

    voltage = readVcc();

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
    uint8_t voltsEncoded = (uint8_t) (voltage / 100);   // eg. 2840 millivolts becomes 28 and in the Dashboard decoded to 2.8 volts

    // read LSM9DS1
    pyfoundationSatLSM9DS1->readGyro();
    pyfoundationSatLSM9DS1->readAccel();
    pyfoundationSatLSM9DS1->readMag();

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
 

   // ***********************************************
   // Prepare message payload. NOTE: The order of the
   // payload data is important for the TTN decoder. 
   // Do not change the order
   // **********************************************

    // Temperature uses a 16bit two's complement with two decimals, so the range is -327.68 to +327.67 degrees
    message.addTemperature(temperature); 
    
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
    pyfoundationSatLMIC->sendSensorPayload(SENSOR_01_SHT31, message);  

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