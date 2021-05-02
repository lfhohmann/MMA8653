/* 
 * @file interrupt.ino
 *
 * @author Lucas Hohmann - @lfhohmann
 *
 * @brief NXP MMA8653 Accelerometer library for Arduino - Interrupt Example
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

#define INT_1_PIN (29) // PIN NUMBER WIRED TO INT1 OUTPUT

MMA8653 accel = MMA8653();
int16_t x, y, z;

bool newAccelData = false;
bool setupCompleted = false;

void accelInt()
{
    if (setupCompleted)
    {
        accel.readSensor(&x, &y, &z);
        newAccelData = true;
    }
}

void setup()
{
    // I2C
    Wire.begin();          // START I2C BUS
    Wire.setClock(400000); // SET I2C CLOCK/SPEED

    // SERIAL
    Serial.begin(9600);

    //PINS
    pinMode(INT_1_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INT_1_PIN), accelInt, RISING);

    // START ACCEL
    accel.init(MMA8653_2G_RANGE, MMA8653_10BIT_RES, MMA8653_ODR_50);
    accel.setInterruptsConfig(MMA8653_INT_PINS_POLARITY_ACTIVE_HIGH, MMA8653_INT_PINS_PUSH_PULL);
    accel.setInterrupt(MMA8653_INT_EN_DRDY, MMA8653_INT_1);
    accel.begin();

    setupCompleted = true;
    accel.readSensor(&x, &y, &z);
}

void loop()
{
    if (newAccelData)
    {
        accel.readSensor(&x, &y, &z);
        Serial.print(x);
        Serial.print(",");
        Serial.print(y);
        Serial.print(",");
        Serial.print(z);
        Serial.println();

        newAccelData = false;
    }
}