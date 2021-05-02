/**
 * @file MMA8653.cpp
 * 
 * @author Lucas Hohmann - @lfhohmann
 * @brief NXP MMA8653 Accelerometer library for Arduino
 * @link https://github.com/lfhohmann/MMA8653
 * @note Only tested on (BBCMicroBit V1.3B)
 * @version 2.0
 * @date 2020-09-12
 * 
 * DATASHEET:
 * @link https://www.nxp.com/docs/en/data-sheet/MMA8653FC.pdf
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "MMA8653.h"
#include "wireAdvanced.h"

/**
 * @brief Construct a new MMA8653 object
 * 
 */
MMA8653::MMA8653()
{
    _is_active = false;
    _was_active = false;
    // SET I2C Address on constructor?
    // Add DataReady Reading function
}

/**
 * @brief Initializes Accelerometer with optional parameters
 * 
 * @param range:      MMA8653_2G_RANGE  = 2G (DEFAULT)
 *                    MMA8653_4G_RANGE  = 4G
 *                    MMA8653_8G_RANGE  = 8G
 * 
 * @param resolution: MMA8653_10BIT_RES = (-512 to 511) RAW outputs (DEFAULT)
 *                    MMA8653_8BIT_RES  = (-128 to 127) RAW outputs 
 *                    
 * @param odr:        MMA8653_ODR_800   =  800hz / 1.25ms (DEFAULT)
 *                    MMA8653_ODR_400   =  400hz /  2.5ms
 *                    MMA8653_ODR_200   =  200hz /    5ms
 *                    MMA8653_ODR_100   =  100hz /   10ms
 *                    MMA8653_ODR_50    =   50hz /   20ms
 *                    MMA8653_ODR_12_5  = 12.5hz /   80ms
 *                    MMA8653_ODR_6_25  = 6.25hz /  160ms
 *                    MMA8653_ODR_1_56  = 1.56hz /  641ms
 * 
 * @return true  = WHO_AM_I response matches, initialization procedure performed 
 * @return false = WHO_AM_I response does't match, initialization procedure aborted
 * 
 * @note This function does not put the Accelerometer in ACTIVE mode, it is necessary to call "begin()"
 * or "exitStandby()" to be able to call "readSensor()".
 * @code
 * MMA8653.init();
 * MMA8653.begin();
 * @endcode
 * 
 * @note It is also recomended that configuration functions be called between "init()" and "begin".
 * @code 
 * MMA8653.init();
 * MMA8653.setDataRate(MMA8653_ODR_50);
 * MMA8653.setInterrupt(MMA8653_INT_EN_DRDY, MMA8653_INT_2);
 * MMA8653.begin();
 * @endcode
 */
bool MMA8653::init(uint8_t range, bool resolution, uint8_t data_rate)
{
    if (!whoAmI())
        return false;

    reset();
    setRange(range);
    setDataRate(data_rate);
    setResolution(resolution);
    return true;
}

/**
 * @brief Check if Accelerometer "WHO_AM_I" register matches the expected response
 * 
 * @return true  = WHO_AM_I response matches
 * @return false = WHO_AM_I does't match
 */
bool MMA8653::whoAmI()
{
    if (readRegister(MMA8653_I2C_ADDRESS, MMA8653_WHO_AM_I_REG) != MMA8653_WHO_AM_I_RSP)
        return false;

    return true;
}

/**
 * @brief Resets Accelerometer to default settings
 * 
 */
void MMA8653::reset()
{
    _is_active = false;

    writeRegister(MMA8653_I2C_ADDRESS, MMA8653_CTRL_REG2, MMA8653_CTRL_REG2_RESET); // Reset device
    delay(10);                                                                      // Wait for it to reset
}

/**
 * @brief Put Accelerometer in STANDBY mode
 * 
 */
void MMA8653::enterStandby()
{
    _is_active = false;
    changeRegister(MMA8653_I2C_ADDRESS, MMA8653_CTRL_REG1, 0b00000001, ~MMA8653_CTRL_REG1_VALUE_ACTIVE);
}

/**
 * @brief Put Accelerometer in ACTIVE mode
 * 
 */
void MMA8653::exitStandby()
{
    changeRegister(MMA8653_I2C_ADDRESS, MMA8653_CTRL_REG1, 0b00000001, MMA8653_CTRL_REG1_VALUE_ACTIVE);
    _is_active = true;
}

/**
 * @brief Start Accelerometer, switch from STANDBY to ACTIVE mode
 * 
 * @note  Just an "alias" for "exitStandby".
 */
void MMA8653::begin()
{
    exitStandby();
}

/**
 * @brief Returns if Accelerometer is in STANDBY or ACTIVE mode
 * 
 * @return true  = Accelerometer is in ACTIVE mode
 * @return false = Accelerometer is in STANDBY mode
 */

bool MMA8653::isActive()
{
    return _is_active;
}

/**
 * @brief Set the Output Data Rate
 * 
 * @param odr: MMA8653_ODR_800  =  800hz / 1.25ms
 *             MMA8653_ODR_400  =  400hz /  2.5ms
 *             MMA8653_ODR_200  =  200hz /    5ms
 *             MMA8653_ODR_100  =  100hz /   10ms
 *             MMA8653_ODR_50   =   50hz /   20ms
 *             MMA8653_ODR_12_5 = 12.5hz /   80ms
 *             MMA8653_ODR_6_25 = 6.25hz /  160ms
 *             MMA8653_ODR_1_56 = 1.56hz /  641ms
 * Example: 
 * @code
 * MMA8653.setDataRate(MMA8653_ODR_50);
 * @endcode
 */
void MMA8653::setDataRate(uint8_t data_rate)
{
    _checkStandby();

    changeRegister(MMA8653_I2C_ADDRESS, MMA8653_CTRL_REG1, MMA8653_CTRL_REG1_ODR_MASK, data_rate);

    _checkStandby();
}

/**
 * @brief Returns current Output Data Rate (ODR)
 * 
 * @return uint8_t ranging from 0x00 to 0x38 @see "MMA8653_ODR" definitions on the Header File
 */
uint8_t MMA8653::getDataRate()
{
    return readRegister(MMA8653_I2C_ADDRESS, MMA8653_CTRL_REG1, MMA8653_CTRL_REG1_ODR_MASK);
}

/**
 * @brief Enable and configure interrupt pins (INT1/INT2)
 * 
 * @param interrupt: MMA8653_INT_EN_ASLP
 *                   MMA8653_INT_EN_LNDPRT
 *                   MMA8653_INT_EN_FF_MT
 *                   MMA8653_INT_EN_DRDY
 * 
 * @param config:    MMA8653_INT_OFF
 *                   MMA8653_INT_1
 *                   MMA8653_INT_2 / (DEFAULT)
 * 
 * @note This sets interrupts individually.
 * 
 * Example: 
 * @code
 * MMA8653.setInterrupt(MMA8653_INT_EN_DRDY, MMA8653_INT_2);
 * @endcode
 */
void MMA8653::setInterrupt(uint8_t interrupt, uint8_t config)
{
    // bool _was_active = _is_active;

    _checkStandby();

    uint8_t reg4 = readRegister(MMA8653_I2C_ADDRESS, MMA8653_CTRL_REG4);
    uint8_t reg5 = readRegister(MMA8653_I2C_ADDRESS, MMA8653_CTRL_REG5);

    switch (config)
    {
    case 0:
        reg4 &= (~interrupt);
        reg5 &= (~interrupt);
        break;
    case 1:
        reg4 |= interrupt;
        reg5 |= interrupt;
        break;
    case 2:
        reg4 |= interrupt;
        reg5 &= (~interrupt);
        break;
    }

    writeRegister(MMA8653_I2C_ADDRESS, MMA8653_CTRL_REG4, reg4);
    writeRegister(MMA8653_I2C_ADDRESS, MMA8653_CTRL_REG5, reg5);

    _checkStandby();
}
//
//
// UINT TO BOOL FOR POLARITY + PUSH PULL?
//
//
//
void MMA8653::setInterruptsConfig(bool polarity, bool pushpull_opendrain)
{
    // bool _was_active = _is_active;

    _checkStandby();

    changeRegister(MMA8653_I2C_ADDRESS, MMA8653_CTRL_REG3, (MMA8653_CTRL_REG3_PP_OD_MASK | MMA8653_CTRL_REG3_IPOL_MASK), (polarity * MMA8653_CTRL_REG3_IPOL_MASK | pushpull_opendrain * MMA8653_CTRL_REG3_PP_OD_MASK));

    _checkStandby();
}
//
//
//
//
//
//

/**
 * @brief Gets the interrupt configuration
 * 
 * @param interrupt: MMA8653_INT_EN_ASLP
 *                   MMA8653_INT_EN_LNDPRT
 *                   MMA8653_INT_EN_FF_MT
 *                   MMA8653_INT_EN_DRDY
 * 
 * @return:          0 = Interrupt disabled
 *                   1 = Interrupt routed to INT1 pin
 *                   2 = Interrupt routed to INT2 pin
 * 
 * Example: 
 * @code
 * MMA8653.getInterrupt(MMA8653_INT_EN_DRDY);
 * @endcode
 */
uint8_t MMA8653::getInterrupt(uint8_t interrupt)
{
    bool reg4 = (readRegister(MMA8653_I2C_ADDRESS, MMA8653_CTRL_REG4) & interrupt) == interrupt;
    uint8_t reg5 = 2 - ((readRegister(MMA8653_I2C_ADDRESS, MMA8653_CTRL_REG5) & interrupt) == interrupt);

    return reg4 * reg5;
}

/**
 * @brief Set to 2G/4G/8G range modes
 * 
 * @param range: MMA8653_2G_RANGE = -2G to +2G
 *               MMA8653_4G_RANGE = -4G to +4G
 *               MMA8653_8G_RANGE = -8G to +8G
 * 
 * Example: 
 * @code
 * MMA8653.setRange(MMA8653_4G_RANGE);
 * @endcode
 */
void MMA8653::setRange(uint8_t range)
{
    _checkStandby();

    if (range == MMA8653_2G_RANGE)
        range = MMA8653_XYZ_DATA_CFG_REG_VALUE_2G;
    else if (range == MMA8653_4G_RANGE)
        range = MMA8653_XYZ_DATA_CFG_REG_VALUE_4G;
    else if (range == MMA8653_8G_RANGE)
        range = MMA8653_XYZ_DATA_CFG_REG_VALUE_8G;

    writeRegister(MMA8653_I2C_ADDRESS, MMA8653_XYZ_DATA_CFG_REG, range);

    _checkStandby();
}

/**
 * @brief Returns current range mode (2G/4G/8G)
 * 
 * @return uint8_t value (2/4/8)
 */
uint8_t MMA8653::getRange()
{
    uint8_t reg_value = readRegister(MMA8653_I2C_ADDRESS, MMA8653_XYZ_DATA_CFG_REG);

    if (reg_value == MMA8653_XYZ_DATA_CFG_REG_VALUE_2G)
        return 2;
    else if (reg_value == MMA8653_XYZ_DATA_CFG_REG_VALUE_4G)
        return 4;
    else if (reg_value == MMA8653_XYZ_DATA_CFG_REG_VALUE_8G)
        return 8;

    return 0;
}

void MMA8653::setMODS(uint8_t active_mode_mods)
{
    _checkStandby();

    changeRegister(MMA8653_I2C_ADDRESS, MMA8653_CTRL_REG2, MMA8653_CTRL_REG2_MODS_MASK, active_mode_mods);

    _checkStandby();
}

void MMA8653::setSMODS(uint8_t sleep_mode_mods)
{
    _checkStandby();

    changeRegister(MMA8653_I2C_ADDRESS, MMA8653_CTRL_REG2, MMA8653_CTRL_REG2_SMODS_MASK, (sleep_mode_mods << 3));

    _checkStandby();
}

/**
 * @brief Set to 8bit or 10bit resolution modes
 * 
 * @param resolution: MMA8653_10BIT_RES = (-512 to 511) RAW outputs
 *                    MMA8653_8BIT_RES  = (-128 to 127) RAW outputs
 *                    
 * 
 * Example: 
 * @code
 * MMA8653.setResolution(MMA8653_8BIT_RES);
 * @endcode
 */
void MMA8653::setResolution(bool resolution)
{
    _checkStandby();

    _resolution = resolution;

    changeRegister(MMA8653_I2C_ADDRESS, MMA8653_CTRL_REG1, MMA8653_CTRL_REG1_VALUE_F_READ_MASK, (!resolution * MMA8653_CTRL_REG1_VALUE_F_READ));

    _checkStandby();
}

/**
 * @brief Returns current resolution mode (8bit/10bit)
 *
 * @return true  = 10Bit Mode
 * @return false = 8Bit Mode
 */
bool MMA8653::getResolution()
{
    return !(readRegister(MMA8653_I2C_ADDRESS, MMA8653_CTRL_REG1, MMA8653_CTRL_REG1_VALUE_F_READ_MASK) == MMA8653_CTRL_REG1_VALUE_F_READ);
}

/**
 * @brief Resets XYZ offsets back to (0,0,0)
 * 
 */
void MMA8653::resetOffsets()
{
    setOffsets(0, 0, 0);
}

/**
 * @brief Set XYZ offsets
 * 
 * @param ox: X axis Offset (-128 to -127)
 * @param oy: Y axis Offset (-128 to -127)
 * @param oz: Z axis Offset (-128 to -127)
 * 
 * @note Offsets are range independent(2G/4G/8G), they will always adjust from -0.25G to +0.25G,
 * which translates to -128 to 127 RAW, about (1.96mg/LSB).
 * 
 * Example: 
 * @code
 * MMA8653.setOffsets( 12, 0, -35);
 * @endcode
 */
void MMA8653::setOffsets(int8_t ox, int8_t oy, int8_t oz)
{
    _checkStandby();

    Wire.beginTransmission((uint8_t)MMA8653_I2C_ADDRESS);
    Wire.write((uint8_t)MMA8653_OFF_X_REG);
    Wire.write((uint8_t)ox);
    Wire.write((uint8_t)oy);
    Wire.write((uint8_t)oz);
    Wire.endTransmission();

    _checkStandby();
}
/**
 * @brief Get XYZ offsets
 * @param ox[out]: Pointer for X axis variable
 * @param oy[out]: Pointer for Y axis variable 
 * @param oz[out]: Pointer for Z axis variable 
 * @note This function doesn't return anything, it stores XYZ values on the variable Pointers
 * passed to it as parameters. These variables should be declared as "int8_t".
 * 
 * Example: 
 * @code 
 * int8_t ox, oy, oz;                   
 * MMA8653.getOffsets(&ox, &oy, &oz);
 * @endcode
 */
void MMA8653::getOffsets(int8_t *ox, int8_t *oy, int8_t *oz)
{
    Wire.beginTransmission((uint8_t)MMA8653_I2C_ADDRESS);
    Wire.write((uint8_t)MMA8653_OFF_X_REG);
    Wire.endTransmission(false);

    Wire.requestFrom((uint8_t)MMA8653_I2C_ADDRESS, 3);

    *ox = Wire.read();
    *oy = Wire.read();
    *oz = Wire.read();
}

/**
 * @brief Set FF_MT completely off and make sure that FF_MT Interrupt is disabled
 * 
 */
void MMA8653::setFFMTOff()
{
    _checkStandby();

    writeRegister(MMA8653_I2C_ADDRESS, MMA8653_FFMT_CFG_REG, MMA8653_FFMT_CFG_REG_OFF);
    setInterrupt(MMA8653_INT_EN_FFMT, MMA8653_INT_OFF);

    _checkStandby();
}

/**
 * 
 * @todo
 *      CHANGE ELE and OAE to bools
 *      CHANGE AXIS to bools (x,y,z) ON/OFF?
 * 
 * @brief Set FF_MT ELE, OAE and enabled axis (XYZ)
 * 
 * @param event_latch:  MMA8653_FFMT_ELE_ON  = Event Latch ON
 *                      MMA8653_FFMT_ELE_OFF = Event Latch OFF
 * 
 * @param or_and_mode:  MMA8653_FFMT_OAE_OR  = Logical OR axis combination mode
 *                      MMA8653_FFMT_OAE_AND = Logical AND axis combination mode
 * 
 * @param enabled_axis: MMA8653_FFMT_DISABLED = NO  axis enabled -> FF_MT is off
 *                      MMA8653_FFMT_AXIS_X   = X   axis
 *                      MMA8653_FFMT_AXIS_XY  = XY  axis
 *                      MMA8653_FFMT_AXIS_XZ  = XZ  axis
 *                      MMA8653_FFMT_AXIS_XYZ = XYZ axis
 *                      MMA8653_FFMT_AXIS_Y   = Y   axis
 *                      MMA8653_FFMT_AXIS_YZ  = YZ  axis
 *                      MMA8653_FFMT_AXIS_Z   = Z   axis
 */
void MMA8653::setFFMTConfig(uint8_t event_latch, uint8_t or_and_mode, uint8_t enabled_axis)
{
    _checkStandby();

    writeRegister(MMA8653_I2C_ADDRESS, MMA8653_FFMT_CFG_REG, (event_latch | or_and_mode | enabled_axis));

    _checkStandby();
}

/**
 * @brief Set FF_MT Threshold and Count
 * 
 * @param threshold: Goes from 0 to 127 = (0G to +-8G)
 * @param count: Goes from 0 to 255 ( @see DataSheet )
 */
void MMA8653::setFFMTThresholds(uint8_t threshold, uint8_t count)
{
    _checkStandby();

    writeRegister(MMA8653_I2C_ADDRESS, MMA8653_FFMT_THS_REG, threshold);
    writeRegister(MMA8653_I2C_ADDRESS, MMA8653_FFMT_COUNT_REG, count);

    _checkStandby();
}

uint8_t MMA8653::readFFMT()
{
    if (!_is_active)
        return 0;

    return readRegister(MMA8653_I2C_ADDRESS, MMA8653_FFMT_SRC_REG);
}

void MMA8653::readFFMTfull(bool *ffmt_ea, int8_t *ffmt_x, int8_t *ffmt_y, int8_t *ffmt_z)
{
    if (!_is_active)
        return;

    uint8_t reg_value = readRegister(MMA8653_I2C_ADDRESS, MMA8653_FFMT_SRC_REG);

    int8_t values[3] = {0, 0, 0};

    for (uint8_t i = 0; i < 3; i++)
    {
        uint8_t bit = (i * 2) + 1;

        if (bitRead(reg_value, bit))
        {
            if (bitRead(reg_value, bit - 1))
                values[i] = -1;
            else
                values[i] = 1;
        }
    }

    *ffmt_ea = bitRead(reg_value, 7);
    *ffmt_x = (int8_t)values[0];
    *ffmt_y = (int8_t)values[1];
    *ffmt_z = (int8_t)values[2];
}

/**
 * @brief Read accelerometer XYZ values
 * @param ax[out]: Pointer for X axis variable
 * @param ay[out]: Pointer for Y axis variable 
 * @param az[out]: Pointer for Z axis variable 
 * @note This function doesn't return anything, it stores XYZ values on the variable Pointers
 * passed to it as parameters. These variables should be declared as "int16_t".
 * 
 * Example: 
 * @code
 * int16_t ax, ay, az;                   
 * MMA8653.readSensor(&ax, &ay, &az);
 * @endcode
 */
void MMA8653::readSensor(int16_t *ax, int16_t *ay, int16_t *az)
{
    if (!_is_active)
        return;

    Wire.beginTransmission((uint8_t)MMA8653_I2C_ADDRESS);
    Wire.write((uint8_t)MMA8653_OUT_X_MSB_REG);
    Wire.endTransmission(false);

    int16_t values[3];

    if (_resolution == MMA8653_10BIT_RES)
        Wire.requestFrom(MMA8653_I2C_ADDRESS, 6);
    else
        Wire.requestFrom(MMA8653_I2C_ADDRESS, 3);

    for (uint8_t idx = 0; idx < 3; idx++)
    {
        values[idx] = Wire.read();

        if (_resolution == MMA8653_10BIT_RES)
        {
            values[idx] <<= 8;
            values[idx] |= Wire.read();
            values[idx] >>= 6;
        }
        else
        {
            values[idx] = (int8_t)values[idx];
        }

        *ax = (int16_t)values[0];
        *ay = (int16_t)values[1];
        *az = (int16_t)values[2];
    }
}

void MMA8653::_checkStandby()
{
    if (_is_active)
    {
        _was_active = true;
        enterStandby();
    }
    else if (_was_active)
    {
        _was_active = false;
        exitStandby();
    }
}