/*
 * SHDLC-Generator: 0.8.2
 * Yaml Version: 0.1.0
 * Template Version: 0.6.0-5-g3f01745
 */
/*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <Arduino.h>
#include <SensirionUartSen44.h>

// Adjust as needed for you Arduino board
#define SENSOR_SERIAL_INTERFACE \
    Serial  // TODO: DRIVER_GENERATOR adjust to Serial1

SensirionUartSen44 sen44;

// TODO: DRIVER_GENERATOR Add missing commands and make prints more pretty

void setup() {
    uint16_t error;
    char errorMessage[256];

    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }

    SENSOR_SERIAL_INTERFACE.begin(115200);
    while (!SENSOR_SERIAL_INTERFACE) {
        delay(100);
    }

    sen44.begin(SENSOR_SERIAL_INTERFACE);

    unsigned char serialNumber[255];
    uint8_t serialNumberSize = 255;

    error = sen44.getSerialNumber(serialNumber, serialNumberSize);

    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("SerialNumber:");
        Serial.println((char*)serialNumber);
    }

    uint8_t firmwareMajor;
    uint8_t firmwareMinor;
    bool firmwareDebug;
    uint8_t hardwareMajor;
    uint8_t hardwareMinor;
    uint8_t protocolMajor;
    uint8_t protocolMinor;

    error = sen44.getVersion(firmwareMajor, firmwareMinor, firmwareDebug,
                             hardwareMajor, hardwareMinor, protocolMajor,
                             protocolMinor);

    if (error) {
        Serial.print("Error trying to execute getVersion(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("FirmwareMajor:");
        Serial.print(firmwareMajor);
        Serial.print("\t");
        Serial.print("FirmwareMinor:");
        Serial.print(firmwareMinor);
        Serial.print("\t");
        Serial.print("FirmwareDebug:");
        Serial.print(firmwareDebug);
        Serial.print("\t");
        Serial.print("HardwareMajor:");
        Serial.print(hardwareMajor);
        Serial.print("\t");
        Serial.print("HardwareMinor:");
        Serial.print(hardwareMinor);
        Serial.print("\t");
        Serial.print("ProtocolMajor:");
        Serial.print(protocolMajor);
        Serial.print("\t");
        Serial.print("ProtocolMinor:");
        Serial.println(protocolMinor);
    }

    // Start Measurement

    error = sen44.startMeasurement();

    if (error) {
        Serial.print("Error trying to execute startMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
}

void loop() {
    uint16_t error;
    char errorMessage[256];

    // TODO: DRIVER_GENERATOR Adjust to correct measurement delay
    delay(1000);
    // Read Measurement
    // TODO: DRIVER_GENERATOR Add scaling and offset to printed measurement
    // values

    uint16_t massConcentrationPm1p0;
    uint16_t massConcentrationPm2p5;
    uint16_t massConcentrationPm4p0;
    uint16_t massConcentrationPm10p0;
    int16_t vocIndex;
    int16_t ambientHumidity;
    int16_t ambientTemperature;

    error = sen44.readMeasuredMassConcentrationAndAmbientValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, vocIndex, ambientHumidity, ambientTemperature);

    if (error) {
        Serial.print("Error trying to execute "
                     "readMeasuredMassConcentrationAndAmbientValues(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("MassConcentrationPm1p0:");
        Serial.print(massConcentrationPm1p0);
        Serial.print("\t");
        Serial.print("MassConcentrationPm2p5:");
        Serial.print(massConcentrationPm2p5);
        Serial.print("\t");
        Serial.print("MassConcentrationPm4p0:");
        Serial.print(massConcentrationPm4p0);
        Serial.print("\t");
        Serial.print("MassConcentrationPm10p0:");
        Serial.print(massConcentrationPm10p0);
        Serial.print("\t");
        Serial.print("VocIndex:");
        Serial.print(vocIndex);
        Serial.print("\t");
        Serial.print("AmbientHumidity:");
        Serial.print(ambientHumidity);
        Serial.print("\t");
        Serial.print("AmbientTemperature:");
        Serial.println(ambientTemperature);
    }
}
