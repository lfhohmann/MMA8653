/* 
 * @file basic.ino
 *
 * @author Lucas Hohmann - @lfhohmann
 *
 * @brief NXP MMA8653 Accelerometer library for Arduino - Basic Example
 * @link https://github.com/lfhohmann/MMA8653
 * @version 1.0
 * @date 2020-09-12
 *
 * DATASHEET:
 * @link https://www.nxp.com/docs/en/data-sheet/MMA8653FC.pdf
 * 
 * @copyright Copyright (c) 2020
 *
 * 
 *
 * Ranges:
 *  2G -> MMA8653_2G_RANGE
 *  4G -> MMA8653_4G_RANGE
 *  8G -> MMA8653_8G_RANGE
 * 
 * Resolutions:
 *  8 bit  -> MMA8653_8BIT_RES
 *  10 bit -> MMA8653_10BIT_RES
 *  
 * Output Data Rate (ODR):
 *  800hz  -> MMA8653_ODR_800
 *  400hz  -> MMA8653_ODR_400
 *  200hz  -> MMA8653_ODR_200
 *  100hz  -> MMA8653_ODR_100
 *  50hz   -> MMA8653_ODR_50
 *  12.5hz -> MMA8653_ODR_12_5
 *  6.25hz -> MMA8653_ODR_6_25
 *  1.56hz -> MMA8653_ODR_1_56
 */

#include <MMA8653.h>

MMA8653 accel = MMA8653();
int16_t x, y, z;

void setup()
{
    // I2C
    Wire.begin();          // START I2C BUS
    Wire.setClock(400000); // SET I2C CLOCK/SPEED

    // SERIAL
    Serial.begin(9600);

    // START ACCEL
    accel.init(MMA8653_2G_RANGE, MMA8653_10BIT_RES, MMA8653_ODR_50);
    accel.begin();
}

void loop()
{
    accel.readSensor(&x, &y, &z);
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.print(z);
    Serial.println();

    delay(20);
}