/**
 * @file wireAdvanced.h
 * 
 * @author Lucas Hohmann
 * @brief Simple tool to be used by other I2C devices libraries
 * @version 0.9
 * @date 2020-09-12
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef wireAdvanced_H
#define wireAdvanced_H

#include <Arduino.h>
#include <Wire.h>

byte readRegister(byte dev_address, byte reg_offset, byte bit_mask = 0b11111111)
{
    Wire.beginTransmission((byte)dev_address);
    Wire.write((byte)reg_offset);
    Wire.endTransmission(false);

    Wire.requestFrom(dev_address, 1);
    if (Wire.available())
        return (Wire.read() & bit_mask);

    return 0;
}

void writeRegister(byte dev_address, byte reg_offset, byte reg_value)
{
    Wire.beginTransmission((byte)dev_address);
    Wire.write((byte)reg_offset);
    Wire.write((byte)reg_value);
    Wire.endTransmission();
}

void changeRegister(byte dev_address, byte reg_offset, byte bit_mask, byte reg_value)
{
    reg_value = (readRegister(dev_address, reg_offset) & ~bit_mask) | reg_value;

    writeRegister(dev_address, reg_offset, reg_value);
}

#endif
