/* 
 * @file offsets.ino
 *
 * @author Lucas Hohmann - lfhohmann@gmail.com
 *
 * @brief NXP MMA8653 Accelerometer library for Arduino - Offsets Example
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
 * The MMA8653 has an 8bit offset value for each axis (X,Y,Z), the offset is
 * range independent (-128 RAW equates to -0.25G and 127 equates to +0.25G),
 * resulting in about (1.96mg/LSB).
 * 
 * This sketch set offsets to X=100, Y=-100, Z=0 during setup(). Then reads
 * both the offset values and the actual accelerometer values in the loop().
 * 
 * 
 * 
 * FUNCTIONS:
 *  resetOffsets()
 *      Resets the offsets values of all axis (X,Y,Z) to 0.
 * 
 *  setOffsets(int8_t ox, int8_t oy, int8_t oz)
 *      Sets the offsets for all axis (X,Y,Z), ranging from -127 to 128.
 *
 *  getOffsets(int8_t *ox, int8_t *oy, int8_t *oz)
 *      Gets the offsets for all axis (X,Y,Z) through pointers. The 3 variables
 *      must be declared first as int8_t.
 */

#include <MMA8653.h>

MMA8653 accel = MMA8653();
int16_t x, y, z;
int8_t offset_x, offset_y, offset_z;

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

    accel.resetOffsets();           // Will make all axis offsets 0
    accel.setOffsets(100, -100, 0); // Set offsets X=100, Y=-100, Z=0

    // Put accelerometer in active mode
    accel.begin();
}

void loop()
{
    // Read Sensor Values and get Offsets
    accel.readSensor(&x, &y, &z);
    accel.getOffsets(&offset_x, &offset_y, &offset_z);

    // Print Offsets
    Serial.print(offset_x);
    Serial.print(",");
    Serial.print(offset_y);
    Serial.print(",");
    Serial.print(offset_z);

    // Print a tab between offsets and values
    Serial.print("\t");

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