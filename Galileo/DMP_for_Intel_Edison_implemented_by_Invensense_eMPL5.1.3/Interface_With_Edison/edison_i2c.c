/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 * $Id: msp430_i2c.h $
 *****************************************************************************/
/**
 *  @defgroup MSP430_System_Layer MSP430 System Layer
 *  @brief  MSP430 System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file       msp430_i2c.c
 *      @brief      Serial communication functions needed by eMPL to
 *                  communicate to the MPU devices.
 *      @details    This driver assumes that eMPL is with a sub-master clock set
 *                  to 20MHz. The following MSP430s are supported:
 *
 *                  MSP430F5528
 *                  MSP430F5529
 */

//#include "msp430.h"
#include "string.h"
#include "edison_setting.h"
#include "edison_i2c.h"
//#include "msp430_clock.h"

//qq95538 Intel Edison mraa i2c operation need an global mraa i2c context for operation.
extern mraa_i2c_context i2c;

//qq95538 use edison instead of msp430.
/*
#define I2C_TIMEOUT_MS  (2500)
#if !defined __MSP430F5438A__ && !defined __MSP430F5528__ && \
    !defined __MSP430F5529__ && !defined __MSP430F5229__
#error  The I2C driver does not support this MSP430.
#endif

#if defined I2C_B0
#define CTL0    UCB0CTL0
#define CTL1    UCB0CTL1
#define IE      UCB0IE
#define BR0     UCB0BR0
#define BR1     UCB0BR1
#define I2CSA   UCB0I2CSA
#define RXBUF   UCB0RXBUF
#define TXBUF   UCB0TXBUF
#define IFG     UCB0IFG
#define I2C_VEC USCI_B0_VECTOR
#define IV      UCB0IV
#elif defined I2C_B1
#define CTL0    UCB1CTL0
#define CTL1    UCB1CTL1
#define IE      UCB1IE
#define BR0     UCB1BR0
#define BR1     UCB1BR1
#define I2CSA   UCB1I2CSA
#define RXBUF   UCB1RXBUF
#define TXBUF   UCB1TXBUF
#define IFG     UCB1IFG
#define I2C_VEC USCI_B1_VECTOR
#define IV      UCB1IV
#else
#error  Define either I2C_B0 or I2C_B1 in your configuration.
#endif

typedef enum {
    STATE_WAITING,
    STATE_READING,
    STATE_WRITING
} msp430_i2c_state;

typedef struct {
    volatile msp430_i2c_state state;
    // First slave register.
    unsigned char slave_reg;
    // 0 if slave register has not been written yet.
    unsigned char slave_reg_written;
    unsigned char *data;
    unsigned short length;
    unsigned char enabled;
} msp430_i2c_info;

static msp430_i2c_info i2c = {
    .enabled = 0
};

#if defined __MSP430F5438A__
#if defined I2C_B0
#define I2C_MODE(void)  do {P3SEL |= 0x06;} while (0)
#define GPIO_MODE(void) do {P3SEL &= ~0x06;} while (0)
#define SET_SCL(void)   do {P3OUT |= 0x04;} while (0)
#define CLEAR_SCL(void) do {P3OUT &= ~0x04;} while (0)
#define SET_SDA(void)   do {P3OUT |= 0x02;} while (0)
#define CLEAR_SDA(void) do {P3OUT &= ~0x02;} while (0)
#else
#define I2C_MODE(void)  do {P5SEL |= 0x10; P3SEL |= 0x80;} while (0)
#define GPIO_MODE(void) do {P5SEL &= ~0x10; P3SEL &= ~0x80;} while (0)
#define SET_SCL(void)   do {P5OUT |= 0x10;} while (0)
#define CLEAR_SCL(void) do {P5OUT &= ~0x10;} while (0)
#define SET_SDA(void)   do {P3OUT |= 0x80;} while (0)
#define CLEAR_SDA(void) do {P3OUT &= ~0x80;} while (0)
#endif
#else
#if defined I2C_B0
#define I2C_MODE(void)  do {P3SEL |= 0x03;} while (0)
#define GPIO_MODE(void) do {P3SEL &= ~0x03;} while (0)
#define SET_SCL(void)   do {P3OUT |= 0x02;} while (0)
#define CLEAR_SCL(void) do {P3OUT &= ~0x02;} while (0)
#define SET_SDA(void)   do {P3OUT |= 0x01;} while (0)
#define CLEAR_SDA(void) do {P3OUT &= ~0x01;} while (0)
#else
#define I2C_MODE(void)  do {P4SEL |= 0x06;} while (0)
#define GPIO_MODE(void) do {P4SEL &= ~0x06;} while (0)
#define SET_SCL(void)   do {P4OUT |= 0x04;} while (0)
#define CLEAR_SCL(void) do {P4OUT &= ~0x04;} while (0)
#define SET_SDA(void)   do {P4OUT |= 0x02;} while (0)
#define CLEAR_SDA(void) do {P4OUT &= ~0x02;} while (0)
#endif
#endif
*/

//qq95538 edison not support enable/disable hardware low-power mode.
/*
int msp430_i2c_enable(void)
{
    unsigned long smclk;
    unsigned short br;
    if (i2c.enabled)
        return 0;
    CTL1 |= UCSWRST;

    I2C_MODE();

    //Set to synchronous master mode.
    CTL0 = UCMST + UCMODE_3 + UCSYNC;

    // Use sub-master clock.
    CTL1 = UCSSEL_2 + UCSWRST;

    // Set slave clock frequency to 400kHz.
    msp430_get_smclk_freq(&smclk);
    br = smclk / 400000L;
    BR0 = (unsigned char)(br & 0xFF);
    BR1 = (unsigned char)((br >> 8) & 0xFF);

    CTL1 &= ~UCSWRST;

    //Enable interrupts.
    IE |= UCTXIE | UCRXIE | UCNACKIE;

    // Initialize struct.
    i2c.state = STATE_WAITING;
    i2c.slave_reg = 0;
    i2c.slave_reg_written = 0;
    i2c.data = 0;
    i2c.length = 0;
    i2c.enabled = 1;

    return 0;
}

int msp430_i2c_disable(void)
{
    if (!i2c.enabled)
        return 0;
    CTL1 |= UCSWRST;
    GPIO_MODE();
    SET_SCL();
    SET_SDA();
    i2c.enabled = 0;
    return 0;
}
*/

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
/*
bool I2Cdev4Edison::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
    //int8_t count = 0;
    uint8_t buf[128];
    //int fd;

    if (length > 127) {
        fprintf(stderr, "Byte write count (%d) > 127\n", length);
        return(FALSE);
    }
    //fd = open("/dev/i2c-0", O_RDWR);
    //if (fd < 0) {
    //    fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
    //    return(FALSE);
    //}
    i2c->address(devAddr);
    //if (ioctl(fd, I2C_SLAVE, devAddr) < 0) {
    //    fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
   //     close(fd);
   //     return(FALSE);
   // }
    buf[0] = regAddr;
    memcpy(buf+1,data,length);
     //count = write(fd, buf, length+1);
    mraa::Result result = i2c->write(buf, length + 1);
    if (result != mraa::SUCCESS){
       	mraa::printError(result);
       	return false;
    }
    return true;

    //if (count < 0) {
    //    fprintf(stderr, "Failed to write device(%d): %s\n", count, ::strerror(errno));
    //    close(fd);
    //    return(FALSE);
    //} else if (count != length+1) {
    //    fprintf(stderr, "Short write to device, expected %d, got %d\n", length+1, count);
    //    close(fd);
    //    return(FALSE);
    //}
    //close(fd);

}
*/
int edison_i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data)
{
	uint8_t buf[128];
	if (length > 127) {
		fprintf(stderr, "MRAA Byte write count (%d) > 127\n", length);
	    return(-1);
	}
	buf[0] = reg_addr;
	memcpy(buf+1,data,length);
	mraa_result_t mr = mraa_i2c_write(i2c, buf, length + 1);
	if (mr != MRAA_SUCCESS){
		fprintf(stderr, "MRAA Failed to write data in reg address %x on the i2c bus.\n", reg_addr);
		mraa_result_print(mr);
		return -1;
	}
	return 0;

	    //if (count < 0) {
	    //    fprintf(stderr, "Failed to write device(%d): %s\n", count, ::strerror(errno));
	    //    close(fd);
	    //    return(FALSE);
	    //} else if (count != length+1) {
	    //    fprintf(stderr, "Short write to device, expected %d, got %d\n", length+1, count);
	    //    close(fd);
	    //    return(FALSE);
	    //}
	    //close(fd);

    /*
	unsigned long start, cur;
    if (!i2c.enabled)
        return -1;
    if (!length)
        return 0;

    // Populate struct.
    i2c.state = STATE_WRITING;
    i2c.slave_reg = reg_addr;
    i2c.slave_reg_written = 0;
    i2c.data = (unsigned char*)data;
    i2c.length = length;

    I2CSA = slave_addr;
    CTL1 |= UCTR | UCTXSTT;

    msp430_get_clock_ms(&start);
    while (i2c.state != STATE_WAITING) {
        __bis_SR_register(LPM0_bits + GIE);
        msp430_get_clock_ms(&cur);
        if (cur >= (start + I2C_TIMEOUT_MS)) {
            CTL1 |= UCTXSTP;
            i2c.state = STATE_WAITING;
            msp430_i2c_disable();
            msp430_delay_ms(1);
            CLEAR_SCL();
            CLEAR_SDA();
            msp430_delay_ms(1);
            SET_SCL();
            SET_SDA();
            msp430_i2c_enable();
            return -1;
        }
    }
    return 0;
    */
}


int  edison_i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data)
{

	int ret = mraa_i2c_read_bytes_data(/*mraa_i2c_context*/ i2c, /*uint8_t command*/reg_addr, /*uint8_t* data*/ data, /*int length*/ length);

	if (ret == length){
		return 0;
	}
	else if (ret == -1){
    	fprintf(stderr, "MRAA i2c bus read bytes operation failed.");
		return -1;
	}
	else if (ret > 0 || ret < length){
		fprintf(stderr, "MRAA i2c the length of bus bytes were read is not equal to which was needed.");
		return -1;
	}
	else{
		fprintf(stderr, "MRAA i2c bus read bytes length error.");
				return -1;
	}

	/*
	unsigned long start, cur;
    if (!i2c.enabled)
        return -1;
    if (!length)
        return 0;

    // Populate struct.
    i2c.state = STATE_READING;
    i2c.slave_reg = reg_addr;
    i2c.slave_reg_written = 0;
    i2c.data = data;
    i2c.length = length;

    I2CSA = slave_addr;
    CTL1 |= UCTR | UCTXSTT;

    msp430_get_clock_ms(&start);
    while (i2c.state != STATE_WAITING) {
        __bis_SR_register(LPM0_bits + GIE);
        msp430_get_clock_ms(&cur);
        if (cur >= (start + I2C_TIMEOUT_MS)) {
            CTL1 |= UCTXSTP;
            i2c.state = STATE_WAITING;
            msp430_i2c_disable();
            msp430_delay_ms(1);
            CLEAR_SCL();
            CLEAR_SDA();
            msp430_delay_ms(1);
            SET_SCL();
            SET_SDA();
            msp430_i2c_enable();
            return -1;
        }
    }
    return 0;
    */
}

/*qq95538 the I2C_ISR code block send and receive data in msp430. Intel edison do not use it but mraa .
#pragma vector = I2C_VEC
__interrupt void I2C_ISR(void)
{
    switch(__even_in_range(IV,12)) {
        case 4:     // NAK interrupt.
            i2c.state = STATE_WAITING;
            CTL1 |= UCTXSTP;
            break;
        case 10:    // RX interrupt.
            IFG &= ~UCRXIFG;
            if (--i2c.length) {
                // If only one byte left, prepare stop signal.
                if (i2c.length == 1)
                    CTL1 |= UCTXSTP;
            } else
                i2c.state = STATE_WAITING;
            // Read RXBUF last because we don't want to release SCL until we're
            // sure we're ready.
            //
            *i2c.data++ = RXBUF;
            break;
        case 12:    // TX interrupt.
            IFG &= ~UCTXIFG;
            switch (i2c.state) {
            case STATE_WRITING:
                if (!i2c.slave_reg_written) {
                    i2c.slave_reg_written = 1;
                    TXBUF = i2c.slave_reg;
                } else if (i2c.length) {
                    // Send next byte, increment pointer.
                    char next = i2c.data[0];
                    i2c.data++;
                    i2c.length--;
                    // Writing to TXBUF must always be the final operation.
                    TXBUF = next;
                } else {
                    i2c.state = STATE_WAITING;
                    CTL1 |= UCTXSTP;
                }
                break;
            case STATE_READING:
                if (!i2c.slave_reg_written) {
                    i2c.slave_reg_written = 1;
                    TXBUF = i2c.slave_reg;
                } else {
                    // Repeated start, switch to RX mode.
                    CTL1 &= ~UCTR;
                    CTL1 |= UCTXSTT;

                    // If single byte, prepare stop signal immediately.
                    if (i2c.length == 1) {
                        // Well, not IMMEDIATELY. First we need to make sure
                        // the start signal got sent.
                        //
                        while (CTL1 & UCTXSTT);
                        CTL1 |= UCTXSTP;
                    }
                }
                break;
            case STATE_WAITING:
            default:
                break;
            }
            break;
        case 0:     // No interrupt.
        case 2:     // Arbitration lost interrupt.
        case 6:     // Start condition interrupt.
        case 8:     // Stop condition interrupt.
        default:
            break;
    }
    __bic_SR_register_on_exit(LPM0_bits);
}
*/
