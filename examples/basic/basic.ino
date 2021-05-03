/* 
 * @file basic.ino
 *
 * @author Lucas Hohmann - lfhohmann@gmail.com
 *
 * @brief NXP MMA8653 Accelerometer library for Arduino - Basic Example
 * @link https://github.com/lfhohmann/MMA8653
 * @version 1.1
 * @date 2020-09-12
 *
 * DATASHEET:
 * @link https://www.nxp.com/docs/en/data-sheet/MMA8653FC.pdf
 * 
 * @copyright Copyright (c) 2020
 *
 * 
 *
 * For basic usage you must first create the MMA8653 object, then call the 
 * init() function with RANGE, RESOLUTION and OUTPUT DATA RATE parameters and,
 * last, call the begin() function to start it up. Just like the example bel-
 * low.
 * 
 * 
 * 
 * FUNCTIONS:
 *  init(uint8_t range, bool resolution, uint8_t data_rate)
 *      Initializes the accelerometer with the specified parameters.
 * 
 *      Ranges:
 *          MMA8653_2G_RANGE -> -2G to +2G
 *          MMA8653_4G_RANGE -> -4G to +4G
 *          MMA8653_8G_RANGE -> -8G to +8G
 * 
 *      Resolutions:
 *          MMA8653_8BIT_RES  -> 8bit
 *          MMA8653_10BIT_RES -> 10bit
 *  
 *      Output Data Rate (ODR):
 *          MMA8653_ODR_800  -> 800hz
 *          MMA8653_ODR_400  -> 400hz
 *          MMA8653_ODR_200  -> 200hz
 *          MMA8653_ODR_100  -> 100hz
 *          MMA8653_ODR_50   -> 50hz
 *          MMA8653_ODR_12_5 -> 12.5hz
 *          MMA8653_ODR_6_25 -> 6.25hz
 *          MMA8653_ODR_1_56 -> 1.56hz
 *
 * 
 *  setMODS(uint8_t active_mode)
 *      Sets to one of the following modes: Normal, Low Noise & Low Power, High
 *      Resolution or Low Power. (Refer to the datasheet).
 * 
 *      Modes (MODS)
 *          MMA8653_MODS_NORMAL              -> Normal
 *          MMA8653_MODS_LOW_NOISE_LOW_POWER -> Low Noise & Low Power
 *          MMA8653_MODS_HIGH_RES            -> High Resolution
 *          MMA8653_MODS_LOW_POWER           -> Low Power
 * 
 * 
 *  begin()
 *      Puts the accelerometer in active mode.
 *  
 * 
 *  readSensor(int16_t *ax, int16_t *ay, int16_t *az)
 *      Reads the sensor X,Y,Z values through pointers. The 3 variables must 
 *      be declared first as int16_t (for both 8bit and 10bit res modes).
 */

#include <MMA8653.h>

MMA8653 accel = MMA8653();
int16_t x, y, z;

void setup()
{
    // Start I2C at 400khz (fast mode)
    Wire.begin();
    Wire.setClock(400000);

    // Start Serial
    Serial.begin(9600);

    // Setup accelerometer at 2G range, 10bit res, 50hz update rate and HighRes mode
    accel.init(MMA8653_2G_RANGE, MMA8653_10BIT_RES, MMA8653_ODR_50);
    accel.setMODS(MMA8653_MODS_HIGH_RES);

    // Put accelerometer in active mode
    accel.begin();
}

void loop()
{
    // Read Sensor Values
    accel.readSensor(&x, &y, &z);

    // Print Sensor Values
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.print(z);
    Serial.println();

    // Wait 20ms until next read (Change this if you change the ODR)
    delay(20);
}