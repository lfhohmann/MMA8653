/* 
 * @file interrupt.ino
 *
 * @author Lucas Hohmann - @lfhohmann
 *
 * @brief NXP MMA8653 Accelerometer library for Arduino - Interrupt Example
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
 * This is a more advanced usage scenario of the library. Instead of polling
 * the sensor every X milliseconds, we wait for it to "notify" us (via an
 * interrupt) and then proceed to read it. This makes it easier to "multitask"
 * while there are no new sensor values.
 * 
 * The sketch attaches the accelInt() function to INT_1_PIN on the micro-
 * controller. The setInterruptsConfig() function sets the polarity to ACTIVE
 * HIGH and PUSH PULL mode (refer to the datasheet). The setInterrupt() function
 * attached the DATA READY interrupt to the MMA8653_INT_1 pin on the MMA8653
 * accelerometer.
 * 
 * 
 * 
 * FUNCTIONS:
 *  setInterruptsConfig(bool polarity, bool pushpull_opendrain)
 *      Configures polarity and pin mode of all interrupt pins.
 * 
 *      polarity:
 *          MMA8653_INT_PINS_POLARITY_ACTIVE_LOW  -> Turn pin HIGH on interrupt
 *          MMA8653_INT_PINS_POLARITY_ACTIVE_HIGH -> Turn pin LOW on interrupt
 *      
 *      pushpull_opendrain:
 *          MMA8653_INT_PINS_PUSH_PULL  -> PushPull mode
 *          MMA8653_INT_PINS_OPEN_DRAIN -> OpenDrain mode
 * 
 * 
 *  setInterrupt(uint8_t interrupt, uint8_t config)
 *      Attach the interrupt type to INT1, INT2 pin or disable it.
 * 
 *      interrupt:
 *          MMA8653_INT_EN_ASLP   -> Auto-SLEEP/WAKE Interrupt
 *          MMA8653_INT_EN_LNDPRT -> Orientation (Landscape/Portrait) Interrupt
 *          MMA8653_INT_EN_FFMT   -> Freefall/Motion Interrupt
 *          MMA8653_INT_EN_DRDY   -> Data Ready Interrupt
 *
 *      config:
 *          MMA8653_INT_OFF -> Disable Interrupt
 *          MMA8653_INT_1   -> Route Interrupt to pin INT1
 *          MMA8653_INT_2   -> Route Interrupt to pin INT2
 */

#include <MMA8653.h>

#define INT_1_PIN (29) // Pin number wired to int1 output (Pin 29 on the BBC MicroBit)

MMA8653 accel = MMA8653();
int16_t x, y, z;

bool newAccelData = false;
bool setupCompleted = false;

// Interrupt attached function
void accelInt()
{
    // Only trigger if setup() has been completed
    if (setupCompleted)
    {
        accel.readSensor(&x, &y, &z);
        newAccelData = true;
    }
}

void setup()
{
    // Start I2C at 400khz (fast mode)
    Wire.begin();
    Wire.setClock(400000);

    // Start Serial
    Serial.begin(9600);

    // Setup Pins
    pinMode(INT_1_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INT_1_PIN), accelInt, RISING);

    // Setup accelerometer at 2G range, 10bit res, 50hz update rate and HighRes mode
    accel.init(MMA8653_2G_RANGE, MMA8653_10BIT_RES, MMA8653_ODR_50);
    accel.setMODS(MMA8653_MODS_HIGH_RES);

    // Setup Interrupt_1 to polarity ACTIVE_HIGH and PUSH_PULL pin mode
    accel.setInterruptsConfig(MMA8653_INT_PINS_POLARITY_ACTIVE_HIGH, MMA8653_INT_PINS_PUSH_PULL);
    accel.setInterrupt(MMA8653_INT_EN_DRDY, MMA8653_INT_1);

    // Put accelerometer in active mode
    accel.begin();

    setupCompleted = true; // Mark setup as completed
}

void loop()
{
    if (newAccelData) // If new accel data available, read it
    {
        accel.readSensor(&x, &y, &z); // Update x,y,z variables with the new values
        Serial.print(x);
        Serial.print(",");
        Serial.print(y);
        Serial.print(",");
        Serial.print(z);
        Serial.println();

        newAccelData = false;
    }
}