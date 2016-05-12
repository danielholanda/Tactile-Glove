/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 * $Id: msp430_i2c.h $  //qq95538 intel edison version edison_i2c.h
 *****************************************************************************/
/**
 *  @defgroup MSP430_System_Layer MSP430 System Layer
 *  @brief  MSP430 System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file       msp430_i2c.h //qq95538 intel edison version edison_i2c.h
 *      @brief      Serial communication functions needed by eMPL to
 *                  communicate to the MPU devices.
 *      @details    This driver assumes that eMPL is with a sub-master clock set
 *                  to 20MHz. The following MSP430s are supported:
 *
 *                  MSP430F5528
 *                  MSP430F5529
 *
 *
 *       qq95538 intel edison control i2c clock frequency as this function
 * 					mraa_result_t mraa_i2c_frequency(mraa_i2c_context dev, mraa_i2c_mode_t mode)
 *
 *
 */
#ifndef _EDISON_I2C_H_
#define _EDISON_I2C_H_

/**
 *  @brief	Set up the I2C port and configure the MSP430 as the master.
 *  @return	0 if successful.
 */
//qq95538 edison not support enable/disable hardware low-power mode.
//int msp_i2c_enable(void);
/**
 *  @brief  Disable I2C communication.
 *  This function will disable the I2C hardware and should be called prior to
 *  entering low-power mode.
 *  @return 0 if successful.
 */
//qq95538 edison not support enable/disable hardware low-power mode.
//int edison_i2c_disable(void);
/**
 *  @brief      Write to a device register.
 *
 *  @param[in]  slave_addr  Slave address of device.
 *  @param[in]  reg_addr	Slave register to be written to.
 *  @param[in]  length      Number of bytes to write.
 *  @param[out] data        Data to be written to register.
 *
 *  @return     0 if successful.
 */
int edison_i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data);
/**
 *  @brief      Read from a device.
 *
 *  @param[in]  slave_addr  Slave address of device.
 *  @param[in]  reg_addr	Slave register to be read from.
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Data from register.
 *
 *  @return     0 if successful.
 */
int edison_i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data);

#endif  /* _EDISON_I2C_H_ */

/**
 * @}
 */
