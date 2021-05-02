/**
 * @file MMA8653.h
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

#ifndef MMA8653_H
#define MMA8653_H

#include <Arduino.h>
#include <Wire.h>

#define MMA8653_I2C_ADDRESS 0x1D
#define MMA8653_WHO_AM_I_REG 0x0D
#define MMA8653_WHO_AM_I_RSP 0x5A

// REGS
#define MMA8653_STATUS_REG 0x00
#define MMA8653_OUT_X_MSB_REG 0x01
#define MMA8653_OUT_X_LSB_REG 0x02
#define MMA8653_OUT_Y_MSB_REG 0x03
#define MMA8653_OUT_Y_LSB_REG 0x04
#define MMA8653_OUT_Z_MSB_REG 0x05
#define MMA8653_OUT_Z_LSB_REG 0x06

// Controls
#define MMA8653_CTRL_REG1 0x2A
#define MMA8653_CTRL_REG1_VALUE_ACTIVE 0x01
#define MMA8653_CTRL_REG1_VALUE_F_READ 0x02
#define MMA8653_CTRL_REG1_VALUE_F_READ_MASK 0x02
#define MMA8653_CTRL_REG1_ODR_MASK 0x38
#define MMA8653_ODR_800 0x00  // 0 - Default ODR (800hz)
#define MMA8653_ODR_400 0x08  // 1 - 400hz
#define MMA8653_ODR_200 0x10  // 2 - 200hz
#define MMA8653_ODR_100 0x18  // 3 - 100hz
#define MMA8653_ODR_50 0x20   // 4 - 50hz
#define MMA8653_ODR_12_5 0x28 // 5 - 12.5hz
#define MMA8653_ODR_6_25 0x30 // 6 - 6.25hz
#define MMA8653_ODR_1_56 0x38 // 7 - 1.56hz

#define MMA8653_CTRL_REG2 0x2B
#define MMA8653_CTRL_REG2_RESET 0x40
#define MMA8653_CTRL_REG2_MODS_MASK 0b00000011
#define MMA8653_CTRL_REG2_SMODS_MASK 0b00011000

#define MMA8653_CTRL_REG3 0x2C
#define MMA8653_CTRL_REG3_PP_OD_MASK 0b00000001
#define MMA8653_CTRL_REG3_PP_OD_VALUE_PP 0x00
#define MMA8653_CTRL_REG3_PP_OD_VALUE_OD 0x01
#define MMA8653_CTRL_REG3_IPOL_MASK 0b00000010
#define MMA8653_CTRL_REG3_IPOL_VALUE_ACTIVE_HIGH 0x02
#define MMA8653_CTRL_REG3_IPOL_VALUE_ACTIVE_LOW 0x00

#define MMA8653_CTRL_REG4 0x2D
#define MMA8653_CTRL_REG4_VALUE_INT_ASLP 0x80
#define MMA8653_CTRL_REG4_VALUE_INT_ENLP 0x10
#define MMA8653_CTRL_REG4_VALUE_INT_FFMT 0x04
#define MMA8653_CTRL_REG4_VALUE_INT_DRDY 0x01

#define MMA8653_CTRL_REG5 0x2E

// Freefall and Motion Detection
#define MMA8653_FF_MT_CFG_REG 0x15
#define MMA8653_FF_MT_CFG_REG_ELE 0x80
#define MMA8653_FF_MT_CFG_REG_OAE 0x40

#define MMA8653_FF_MT_EN_AXES_X 0x08
#define MMA8653_FF_MT_EN_AXES_XY 0x18
#define MMA8653_FF_MT_EN_AXES_XZ 0x28
#define MMA8653_FF_MT_EN_AXES_XYZ 0x38
#define MMA8653_FF_MT_EN_AXES_Y 0x10
#define MMA8653_FF_MT_EN_AXES_YZ 0x30
#define MMA8653_FF_MT_EN_AXES_Z 0x20

#define MMA8653_FF_MT_SRC_REG 0x16
#define MMA8653_FF_MT_SRC_REG_EA 0x80

#define MMA8653_FF_MT_THS_REG 0x17
#define MMA8653_FF_MT_THS_REG_DBCNTM 0x80

#define MMA8653_FF_MT_COUNT_REG 0x18

// Offsets
#define MMA8653_OFF_X_REG 0x2F
#define MMA8653_OFF_Y_REG 0x30
#define MMA8653_OFF_Z_REG 0x31

// Modes
#define MMA8653_XYZ_DATA_CFG_REG 0x0E
#define MMA8653_XYZ_DATA_CFG_REG_VALUE_2G 0x00
#define MMA8653_XYZ_DATA_CFG_REG_VALUE_4G 0x01
#define MMA8653_XYZ_DATA_CFG_REG_VALUE_8G 0x02

#define MMA8653_2G_RANGE 0x02 // Set Range to 2g
#define MMA8653_4G_RANGE 0x04 // Set Range to 4g
#define MMA8653_8G_RANGE 0x08 // Set Range to 8g

// FF_MT
#define MMA8653_FFMT_CFG_REG 0x15
#define MMA8653_FFMT_CFG_REG_ELE 0x80
#define MMA8653_FFMT_CFG_REG_OAE 0x40
#define MMA8653_FFMT_CFG_REG_OFF 0x00

#define MMA8653_FFMT_SRC_REG 0x16
#define MMA8653_FFMT_THS_REG 0x17
#define MMA8653_FFMT_COUNT_REG 0x18

////////USER////////
//ff_mt
#define MMA8653_FFMT_ELE_ON 0b10000000
#define MMA8653_FFMT_ELE_OFF 0b00000000
#define MMA8653_FFMT_OAE_OR 0b01000000
#define MMA8653_FFMT_OAE_AND 0b00000000

#define MMA8653_FFMT_AXIS_X 0b00001000
#define MMA8653_FFMT_AXIS_XY 0b00011000
#define MMA8653_FFMT_AXIS_XZ 0b00101000
#define MMA8653_FFMT_AXIS_XYZ 0b00111000
#define MMA8653_FFMT_AXIS_Y 0b00010000
#define MMA8653_FFMT_AXIS_YZ 0b00110000
#define MMA8653_FFMT_AXIS_Z 0b00100000

//Resolution
#define MMA8653_8BIT_RES 0x00
#define MMA8653_10BIT_RES 0x01

//Mods
#define MMA8653_MODS_NORMAL 0x00
#define MMA8653_MODS_LOW_NOISE_LOW_POWER 0x01
#define MMA8653_MODS_HIGH_RES 0x02
#define MMA8653_MODS_LOW_POWER 0x03

// Interrupts
#define MMA8653_INT_EN_ASLP 0x80   // Auto-SLEEP/WAKE Interrupt
#define MMA8653_INT_EN_LNDPRT 0x10 // Orientation (Landscape/Portrait) Interrupt
#define MMA8653_INT_EN_FFMT 0x04   // Freefall/Motion Interrupt
#define MMA8653_INT_EN_DRDY 0x01   // Data Ready Interrupt

#define MMA8653_INT_OFF 0x00 // Disable Interrupt
#define MMA8653_INT_1 0x01   // Route Interrupt to pin INT1
#define MMA8653_INT_2 0x02   // Route Interrupt to pin INT2

#define MMA8653_INT_PINS_PUSH_PULL 0x00
#define MMA8653_INT_PINS_OPEN_DRAIN 0x01
#define MMA8653_INT_PINS_POLARITY_ACTIVE_LOW 0x00
#define MMA8653_INT_PINS_POLARITY_ACTIVE_HIGH 0x01

class MMA8653
{
public:
    MMA8653();

    //SYSTEM
    bool whoAmI();
    void reset();
    void enterStandby();
    void exitStandby();

    bool init(uint8_t range = MMA8653_2G_RANGE, bool resolution = MMA8653_8BIT_RES, uint8_t data_rate = MMA8653_ODR_100);

    //CONFIGS
    void setRange(uint8_t range);
    uint8_t getRange();

    void setMODS(uint8_t active_mode);
    void setSMODS(uint8_t sleep_mode);

    void setResolution(bool resolution);
    bool getResolution();

    void setDataRate(uint8_t data_rate);
    uint8_t getDataRate();

    void setInterrupt(uint8_t interrupt, uint8_t config);
    uint8_t getInterrupt(uint8_t interrupt);

    void setInterruptsConfig(bool polarity, bool pushpull_opendrain);

    void resetOffsets();
    void setOffsets(int8_t ox, int8_t oy, int8_t oz);
    void getOffsets(int8_t *ox, int8_t *oy, int8_t *oz);

    void setFFMTOff();
    void setFFMTConfig(uint8_t event_latch, uint8_t or_and_mode, uint8_t enabled_axis);
    void setFFMTThresholds(uint8_t threshold, uint8_t count);
    void readFFMTfull(bool *ffmt_ea, int8_t *ffmt_x, int8_t *ffmt_y, int8_t *ffmt_z);
    uint8_t readFFMT();

    //ACTIVE
    void begin();

    void readSensor(int16_t *ax, int16_t *ay, int16_t *az);
    bool isActive();

    uint8_t readREG(uint8_t reg_offset);

private:
    void _checkStandby();

    bool _is_active;
    bool _was_active;
    bool _resolution;
};

#endif
