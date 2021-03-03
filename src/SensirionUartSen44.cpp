/*
 * THIS FILE IS AUTOMATICALLY GENERATED
 *
 * SHDLC-Generator: 0.8.2
 * Yaml Version: 0.1.0
 * Template Version: 0.7.0
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

#include "SensirionUartSen44.h"
#include "Arduino.h"
#include "SensirionCore.h"

#define SEN44_UART_ADDRESS 0x00

SensirionUartSen44::SensirionUartSen44() {
}

void SensirionUartSen44::begin(Stream& serial) {
    _serial = &serial;
}

uint16_t SensirionUartSen44::startMeasurement() {
    uint16_t error;
    uint8_t buffer[12];
    SensirionShdlcTxFrame txFrame(buffer, 12);
    SensirionShdlcRxFrame rxFrame(buffer, 12);

    error = txFrame.begin(0x00, SEN44_UART_ADDRESS, 1);
    error |= txFrame.addUInt8(0x02);
    error |= txFrame.finish();
    if (error) {
        return error;
    }

    error = SensirionShdlcCommunication::sendAndReceiveFrame(*_serial, txFrame,
                                                             rxFrame, 20000);

    return error;
}

uint16_t SensirionUartSen44::stopMeasurement() {
    uint16_t error;
    uint8_t buffer[12];
    SensirionShdlcTxFrame txFrame(buffer, 12);
    SensirionShdlcRxFrame rxFrame(buffer, 12);

    error = txFrame.begin(0x01, SEN44_UART_ADDRESS, 0);
    error |= txFrame.finish();
    if (error) {
        return error;
    }

    error = SensirionShdlcCommunication::sendAndReceiveFrame(*_serial, txFrame,
                                                             rxFrame, 120000);

    return error;
}

uint16_t SensirionUartSen44::readDataReady(bool& dataReady) {
    uint16_t error;
    uint8_t buffer[14];
    SensirionShdlcTxFrame txFrame(buffer, 14);
    SensirionShdlcRxFrame rxFrame(buffer, 14);

    error = txFrame.begin(0x02, SEN44_UART_ADDRESS, 0);
    error |= txFrame.finish();
    if (error) {
        return error;
    }

    error = SensirionShdlcCommunication::sendAndReceiveFrame(*_serial, txFrame,
                                                             rxFrame, 5000);
    if (error) {
        return error;
    }

    error |= rxFrame.getBool(dataReady);
    return error;
}

uint16_t SensirionUartSen44::readMeasuredPmValues(
    uint16_t& massConcentrationPm1p0, uint16_t& massConcentrationPm2p5,
    uint16_t& massConcentrationPm4p0, uint16_t& massConcentrationPm10p0,
    uint16_t& numberConcentrationPm0p5, uint16_t& numberConcentrationPm1p0,
    uint16_t& numberConcentrationPm2p5, uint16_t& numberConcentrationPm4p0,
    uint16_t& numberConcentrationPm10p0, uint16_t& typicalParticleSize) {
    uint16_t error;
    uint8_t buffer[52];
    SensirionShdlcTxFrame txFrame(buffer, 52);
    SensirionShdlcRxFrame rxFrame(buffer, 52);

    error = txFrame.begin(0x03, SEN44_UART_ADDRESS, 1);
    error |= txFrame.addUInt8(0x05);
    error |= txFrame.finish();
    if (error) {
        return error;
    }

    error = SensirionShdlcCommunication::sendAndReceiveFrame(*_serial, txFrame,
                                                             rxFrame, 10000);
    if (error) {
        return error;
    }

    error |= rxFrame.getUInt16(massConcentrationPm1p0);
    error |= rxFrame.getUInt16(massConcentrationPm2p5);
    error |= rxFrame.getUInt16(massConcentrationPm4p0);
    error |= rxFrame.getUInt16(massConcentrationPm10p0);
    error |= rxFrame.getUInt16(numberConcentrationPm0p5);
    error |= rxFrame.getUInt16(numberConcentrationPm1p0);
    error |= rxFrame.getUInt16(numberConcentrationPm2p5);
    error |= rxFrame.getUInt16(numberConcentrationPm4p0);
    error |= rxFrame.getUInt16(numberConcentrationPm10p0);
    error |= rxFrame.getUInt16(typicalParticleSize);
    return error;
}

uint16_t SensirionUartSen44::readMeasuredMassConcentrationAndAmbientValuesTicks(
    uint16_t& massConcentrationPm1p0, uint16_t& massConcentrationPm2p5,
    uint16_t& massConcentrationPm4p0, uint16_t& massConcentrationPm10p0,
    int16_t& vocIndex, int16_t& ambientHumidity, int16_t& ambientTemperature) {
    uint16_t error;
    uint8_t buffer[40];
    SensirionShdlcTxFrame txFrame(buffer, 40);
    SensirionShdlcRxFrame rxFrame(buffer, 40);

    error = txFrame.begin(0x03, SEN44_UART_ADDRESS, 1);
    error |= txFrame.addUInt8(0x07);
    error |= txFrame.finish();
    if (error) {
        return error;
    }

    error = SensirionShdlcCommunication::sendAndReceiveFrame(*_serial, txFrame,
                                                             rxFrame, 10000);
    if (error) {
        return error;
    }

    error |= rxFrame.getUInt16(massConcentrationPm1p0);
    error |= rxFrame.getUInt16(massConcentrationPm2p5);
    error |= rxFrame.getUInt16(massConcentrationPm4p0);
    error |= rxFrame.getUInt16(massConcentrationPm10p0);
    error |= rxFrame.getInt16(vocIndex);
    error |= rxFrame.getInt16(ambientHumidity);
    error |= rxFrame.getInt16(ambientTemperature);
    return error;
}

uint16_t SensirionUartSen44::readMeasuredMassConcentrationAndAmbientValues(
    uint16_t& massConcentrationPm1p0, uint16_t& massConcentrationPm2p5,
    uint16_t& massConcentrationPm4p0, uint16_t& massConcentrationPm10p0,
    float& vocIndex, float& ambientHumidity, float& ambientTemperature) {
    uint16_t error;
    int16_t vocIndexTicks;
    int16_t ambientHumidityTicks;
    int16_t ambientTemperatureTicks;

    error = readMeasuredMassConcentrationAndAmbientValuesTicks(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, vocIndexTicks, ambientHumidityTicks,
        ambientTemperatureTicks);
    if (error) {
        return error;
    }

    vocIndex = static_cast<float>(vocIndexTicks) / 10.0f;
    ambientHumidity = static_cast<float>(ambientHumidityTicks) / 100.0f;
    ambientTemperature = static_cast<float>(ambientTemperatureTicks) / 200.0f;
    return NoError;
}

uint16_t SensirionUartSen44::readMeasuredAmbientValuesTicks(
    int16_t& vocIndex, int16_t& ambientHumidity, int16_t& ambientTemperature) {
    uint16_t error;
    uint8_t buffer[24];
    SensirionShdlcTxFrame txFrame(buffer, 24);
    SensirionShdlcRxFrame rxFrame(buffer, 24);

    error = txFrame.begin(0x03, SEN44_UART_ADDRESS, 1);
    error |= txFrame.addUInt8(0x0A);
    error |= txFrame.finish();
    if (error) {
        return error;
    }

    error = SensirionShdlcCommunication::sendAndReceiveFrame(*_serial, txFrame,
                                                             rxFrame, 10000);
    if (error) {
        return error;
    }

    error |= rxFrame.getInt16(vocIndex);
    error |= rxFrame.getInt16(ambientHumidity);
    error |= rxFrame.getInt16(ambientTemperature);
    return error;
}

uint16_t SensirionUartSen44::readMeasuredAmbientValues(
    float& vocIndex, float& ambientHumidity, float& ambientTemperature) {
    uint16_t error;
    int16_t vocIndexTicks;
    int16_t ambientHumidityTicks;
    int16_t ambientTemperatureTicks;

    error = readMeasuredAmbientValuesTicks(vocIndexTicks, ambientHumidityTicks,
                                           ambientTemperatureTicks);
    if (error) {
        return error;
    }

    vocIndex = static_cast<float>(vocIndexTicks) / 10.0f;
    ambientHumidity = static_cast<float>(ambientHumidityTicks) / 100.0f;
    ambientTemperature = static_cast<float>(ambientTemperatureTicks) / 200.0f;
    return NoError;
}

uint16_t SensirionUartSen44::startFanCleaning() {
    uint16_t error;
    uint8_t buffer[12];
    SensirionShdlcTxFrame txFrame(buffer, 12);
    SensirionShdlcRxFrame rxFrame(buffer, 12);

    error = txFrame.begin(0x56, SEN44_UART_ADDRESS, 0);
    error |= txFrame.finish();
    if (error) {
        return error;
    }

    error = SensirionShdlcCommunication::sendAndReceiveFrame(*_serial, txFrame,
                                                             rxFrame, 20000);

    return error;
}

uint16_t SensirionUartSen44::setAutoCleaningInterval(uint32_t interval) {
    uint16_t error;
    uint8_t buffer[20];
    SensirionShdlcTxFrame txFrame(buffer, 20);
    SensirionShdlcRxFrame rxFrame(buffer, 20);

    error = txFrame.begin(0x80, SEN44_UART_ADDRESS, 5);
    error |= txFrame.addUInt8(0x00);
    error |= txFrame.addUInt32(interval);
    error |= txFrame.finish();
    if (error) {
        return error;
    }

    error = SensirionShdlcCommunication::sendAndReceiveFrame(*_serial, txFrame,
                                                             rxFrame, 100000);

    return error;
}

uint16_t SensirionUartSen44::getAutoCleaningInterval(uint32_t& interval) {
    uint16_t error;
    uint8_t buffer[20];
    SensirionShdlcTxFrame txFrame(buffer, 20);
    SensirionShdlcRxFrame rxFrame(buffer, 20);

    error = txFrame.begin(0x80, SEN44_UART_ADDRESS, 1);
    error |= txFrame.addUInt8(0x00);
    error |= txFrame.finish();
    if (error) {
        return error;
    }

    error = SensirionShdlcCommunication::sendAndReceiveFrame(*_serial, txFrame,
                                                             rxFrame, 20000);
    if (error) {
        return error;
    }

    error |= rxFrame.getUInt32(interval);
    return error;
}

uint16_t SensirionUartSen44::getArticleCode(unsigned char articleCode[],
                                            uint8_t articleCodeSize) {
    uint16_t error;
    uint8_t buffer[76];
    SensirionShdlcTxFrame txFrame(buffer, 76);
    SensirionShdlcRxFrame rxFrame(buffer, 76);

    error = txFrame.begin(0xD0, SEN44_UART_ADDRESS, 1);
    error |= txFrame.addUInt8(0x02);
    error |= txFrame.finish();
    if (error) {
        return error;
    }

    error = SensirionShdlcCommunication::sendAndReceiveFrame(*_serial, txFrame,
                                                             rxFrame, 20000);
    if (error) {
        return error;
    }

    error |= rxFrame.getBytes(articleCode, articleCodeSize);
    return error;
}

uint16_t SensirionUartSen44::getSerialNumber(unsigned char serialNumber[],
                                             uint8_t serialNumberSize) {
    uint16_t error;
    uint8_t buffer[76];
    SensirionShdlcTxFrame txFrame(buffer, 76);
    SensirionShdlcRxFrame rxFrame(buffer, 76);

    error = txFrame.begin(0xD0, SEN44_UART_ADDRESS, 1);
    error |= txFrame.addUInt8(0x03);
    error |= txFrame.finish();
    if (error) {
        return error;
    }

    error = SensirionShdlcCommunication::sendAndReceiveFrame(*_serial, txFrame,
                                                             rxFrame, 20000);
    if (error) {
        return error;
    }

    error |= rxFrame.getBytes(serialNumber, serialNumberSize);
    return error;
}

uint16_t
SensirionUartSen44::getVersion(uint8_t& firmwareMajor, uint8_t& firmwareMinor,
                               bool& firmwareDebug, uint8_t& hardwareMajor,
                               uint8_t& hardwareMinor, uint8_t& protocolMajor,
                               uint8_t& protocolMinor) {
    uint16_t error;
    uint8_t buffer[26];
    SensirionShdlcTxFrame txFrame(buffer, 26);
    SensirionShdlcRxFrame rxFrame(buffer, 26);

    error = txFrame.begin(0xD1, SEN44_UART_ADDRESS, 0);
    error |= txFrame.finish();
    if (error) {
        return error;
    }

    error = SensirionShdlcCommunication::sendAndReceiveFrame(*_serial, txFrame,
                                                             rxFrame, 20000);
    if (error) {
        return error;
    }

    error |= rxFrame.getUInt8(firmwareMajor);
    error |= rxFrame.getUInt8(firmwareMinor);
    error |= rxFrame.getBool(firmwareDebug);
    error |= rxFrame.getUInt8(hardwareMajor);
    error |= rxFrame.getUInt8(hardwareMinor);
    error |= rxFrame.getUInt8(protocolMajor);
    error |= rxFrame.getUInt8(protocolMinor);
    return error;
}

uint16_t SensirionUartSen44::readDeviceStatus(bool clear,
                                              uint32_t& deviceStatus,
                                              uint8_t& lastError) {
    uint16_t error;
    uint8_t buffer[22];
    SensirionShdlcTxFrame txFrame(buffer, 22);
    SensirionShdlcRxFrame rxFrame(buffer, 22);

    error = txFrame.begin(0xD2, SEN44_UART_ADDRESS, 1);
    error |= txFrame.addBool(clear);
    error |= txFrame.finish();
    if (error) {
        return error;
    }

    error = SensirionShdlcCommunication::sendAndReceiveFrame(*_serial, txFrame,
                                                             rxFrame, 20000);
    if (error) {
        return error;
    }

    error |= rxFrame.getUInt32(deviceStatus);
    error |= rxFrame.getUInt8(lastError);
    return error;
}

uint16_t SensirionUartSen44::deviceReset() {
    uint16_t error;
    uint8_t buffer[12];
    SensirionShdlcTxFrame txFrame(buffer, 12);
    SensirionShdlcRxFrame rxFrame(buffer, 12);

    error = txFrame.begin(0xD3, SEN44_UART_ADDRESS, 0);
    error |= txFrame.finish();
    if (error) {
        return error;
    }

    error = SensirionShdlcCommunication::sendAndReceiveFrame(*_serial, txFrame,
                                                             rxFrame, 20000);

    delay(100);
    return error;
}
