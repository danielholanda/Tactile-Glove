/**
 *   @defgroup  eMPL
 *   @brief     Embedded Motion Processing Library
 *
 *   @{
 *       @file      motion_driver_test.c
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


//qq95538 replace msp430 by edison settings.
#include "Interface_With_Edison/edison_setting.h"
#include "Interface_With_Edison/edison_clock.h"
//#include "msp430.h"
//#include "msp430_clock.h"
//#include "msp430_i2c.h"

#include "eMPL/inv_mpu.h"
#include "eMPL/inv_mpu_dmp_motion_driver.h"

/* Data requested by client. */
//qq95538 these types are communication code defined. no value.
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
//qq95538 valuable?
#define DEFAULT_MPU_HZ  (100)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)  //qq95538 £¿

struct rx_s {
    /*msp430 unsigned char header[3];*/
    /*msp430 unsigned char */ char cmd;
};//qq95538 these types are communication code defined. no value.
struct hal_s { //qq95538 a KEY TYPE to control this program.
    unsigned char sensors; //qq95538 these are projected to inv_mpu enums.
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;  //qq95538 especially this one which is used in inv_mpu functions.
    unsigned char motion_int_mode;
    struct rx_s rx;
};//qq95538 these types are communication code defined. no value.
static struct hal_s hal = {0};

//qq95538 declare a global i2c context for mraa i2c operation.
mraa_i2c_context i2c;


/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
//qq95538 do not use usb rx send/receive.
//volatile unsigned char rx_new;

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

enum packet_type_e {
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_GYRO,
    PACKET_TYPE_QUAT,
    PACKET_TYPE_TAP,
    PACKET_TYPE_ANDROID_ORIENT,
    PACKET_TYPE_PEDO,
    PACKET_TYPE_MISC
};

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

/* Handle sensor on/off combinations. */
static void setup_gyro(void)
{
    unsigned char mask = 0;
    if (hal.sensors & ACCEL_ON)
        mask |= INV_XYZ_ACCEL;
    if (hal.sensors & GYRO_ON)
        mask |= INV_XYZ_GYRO;
    /* If you need a power transition, this function should be called with a
     * mask of the sensors still enabled. The driver turns off any sensors
     * excluded from this mask.
     */
    mpu_set_sensors(mask); //qq95538 hal.sensors masks are projected to INV MASKS, and are used in inv_mpu functions.
    if (!hal.dmp_on)
        mpu_configure_fifo(mask);
}

static void tap_cb(unsigned char direction, unsigned char count)
{
	//qq95538 avoid message transfer in callback function.
    /*
	char data[2];
    data[0] = (char)direction;
    data[1] = (char)count;
    send_packet(PACKET_TYPE_TAP, data);
    */
}


static inline void edison_reset(void)
{
	//msp430 code.
    //PMMCTL0 |= PMMSWPOR;
	mraa_deinit();
	fprintf(stdout, "Edison has no reset function, I restart mraa instead.");
	mraa_init();
}

static inline void run_self_test(void)
{
    int result;
    //qq95538 avoid packet transporting in different functions similar to threads.
    //char test_packet[4] = {0};
    long gyro[3], accel[3];
    unsigned char i = 0;

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 2048.f; //convert to +-16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
    }
    //qq95538 avoid meassage transfer in this function.

    /* Report results. */
    /*
    test_packet[0] = 't';
    test_packet[1] = result;
    send_packet(PACKET_TYPE_MISC, test_packet);
    */
    //instead of print result
    printf("mpu_run_self_test() result is: %x \n 0x7d means test passed.\n", result);
}

static void handle_input(/*msp430 void*/char c)
{
	//qq95538 avoid inv command reading.
    /*char c;
    const unsigned char header[3] = "inv";*/
    unsigned long pedo_packet[2];

    /* Read incoming byte and check for header.
     * Technically, the MSP430 USB stack can handle more than one byte at a
     * time. This example allows for easily switching to UART if porting to a
     * different microcontroller.
     */
	/*
    rx_new = 0;
    cdcReceiveDataInBuffer((BYTE*)&c, 1, CDC0_INTFNUM);
    if (hal.rx.header[0] == header[0]) {
        if (hal.rx.header[1] == header[1]) {
            if (hal.rx.header[2] == header[2]) {
                memset(&hal.rx.header, 0, sizeof(hal.rx.header));
                hal.rx.cmd = c;
            } else if (c == header[2])
                hal.rx.header[2] = c;
            else
                memset(&hal.rx.header, 0, sizeof(hal.rx.header));
        } else if (c == header[1])
            hal.rx.header[1] = c;
        else
            memset(&hal.rx.header, 0, sizeof(hal.rx.header));
    } else if (c == header[0])
        hal.rx.header[0] = header[0];
    if (!hal.rx.cmd)
        return;
	*/
	hal.rx.cmd = c;//qq95538 replaces msp430 usb receive code block by scanf outside of this function.
    switch (hal.rx.cmd) {
    /* These commands turn the hardware sensors on/off. */
    case '8':
        if (!hal.dmp_on) {
            /* Accel and gyro need to be on for the DMP features to work
             * properly.
             */
            hal.sensors ^= ACCEL_ON;//qq95538 It seems that hal is a global variable that setup_gyro() use
            setup_gyro();
        }
        break;
    case '9':
        if (!hal.dmp_on) {
            hal.sensors ^= GYRO_ON;
            setup_gyro();
        }
        break;
    /* The commands start/stop sending data to the client. */
    case 'a':
        hal.report ^= PRINT_ACCEL;
        break;
    case 'g':
        hal.report ^= PRINT_GYRO;
        break;
    case 'q':
        hal.report ^= PRINT_QUAT;
        break;
    /* The hardware self test can be run without any interaction with the
     * MPL since it's completely localized in the gyro driver. Logging is
     * assumed to be enabled; otherwise, a couple LEDs could probably be used
     * here to display the test results.
     */
    case 't':
        run_self_test();
        break;
    /* Depending on your application, sensor data may be needed at a faster or
     * slower rate. These commands can speed up or slow down the rate at which
     * the sensor data is pushed to the MPL.
     *
     * In this example, the compass rate is never changed.
     */
    case '1':
        if (hal.dmp_on) //qq95538 To change data push rate maybe redeem the crash of mpu6050 in the i2clib version
            dmp_set_fifo_rate(10);
        else
            mpu_set_sample_rate(10);
        break;
    case '2':
        if (hal.dmp_on)
            dmp_set_fifo_rate(20);
        else
            mpu_set_sample_rate(20);
        break;
    case '3':
        if (hal.dmp_on)
            dmp_set_fifo_rate(40);
        else
            mpu_set_sample_rate(40);
        break;
    case '4':
        if (hal.dmp_on)
            dmp_set_fifo_rate(50);
        else
            mpu_set_sample_rate(50);
        break;
    case '5':
        if (hal.dmp_on)
            dmp_set_fifo_rate(100);
        else
            mpu_set_sample_rate(100);
        break;
    case '6':
        if (hal.dmp_on)
            dmp_set_fifo_rate(200);
        else
            mpu_set_sample_rate(200);
        break;
	case ',':
        /* Set hardware to interrupt on gesture event only. This feature is
         * useful for keeping the MCU asleep until the DMP detects as a tap or
         * orientation event.
         */
        dmp_set_interrupt_mode(DMP_INT_GESTURE);
        break;
    case '.':
        /* Set hardware to interrupt periodically. */
        dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
        break;
    case '7':
        /* Reset pedometer. */
        dmp_set_pedometer_step_count(0);
        dmp_set_pedometer_walk_time(0);
        break;
    case 'f':
        /* Toggle DMP. */
        if (hal.dmp_on) {//qq95538 what is a toggle operation?
            unsigned short dmp_rate;
            hal.dmp_on = 0;
            mpu_set_dmp_state(0);
            /* Restore FIFO settings. */
            mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
            /* When the DMP is used, the hardware sampling rate is fixed at
             * 200Hz, and the DMP is configured to downsample the FIFO output
             * using the function dmp_set_fifo_rate. However, when the DMP is
             * turned off, the sampling rate remains at 200Hz. This could be
             * handled in inv_mpu.c, but it would need to know that
             * inv_mpu_dmp_motion_driver.c exists. To avoid this, we'll just
             * put the extra logic in the application layer.
             */
            dmp_get_fifo_rate(&dmp_rate);
            mpu_set_sample_rate(dmp_rate);
        } else {
            unsigned short sample_rate;
            hal.dmp_on = 1;
            /* Both gyro and accel must be on. */
            hal.sensors |= ACCEL_ON | GYRO_ON;
            mpu_set_sensors(INV_XYZ_ACCEL | INV_XYZ_GYRO);
            mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
            /* Preserve current FIFO rate. */
            mpu_get_sample_rate(&sample_rate);
            dmp_set_fifo_rate(sample_rate);
            mpu_set_dmp_state(1);
        }
        break;
    case 'm':
        /* Test the motion interrupt hardware feature. */
		#ifndef MPU6050 // not enabled for 6050 product
		hal.motion_int_mode = 1;
		#endif
        break;
    case 'p':
        /* Read current pedometer count. */
        dmp_get_pedometer_step_count(pedo_packet);
        dmp_get_pedometer_walk_time(pedo_packet + 1);
        //qq95538 avoid message transfer operation instead of print data.
        //send_packet(PACKET_TYPE_PEDO, pedo_packet);
        printf("PACKET_TYPE_PEDO count:%ld, walk time:%ld.\n", pedo_packet[0], pedo_packet[1]);
        break;
    case 'x':
        edison_reset();
        break;
    case 'v':
        /* Toggle LP quaternion.
         * The DMP features can be enabled/disabled at runtime. Use this same
         * approach for other features.
         */
        hal.dmp_features ^= DMP_FEATURE_6X_LP_QUAT;
        dmp_enable_feature(hal.dmp_features); //qq95538 a KEY function to set dmp.
        break;
    default:
        break;
    }
    hal.rx.cmd = 0;
}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
static void gyro_data_ready_cb(void* args)
{
    hal.new_gyro = 1;
}

/* Set up MSP430 peripherals. */
/*static inline void platform_init(void)
{
	WDTCTL = WDTPW | WDTHOLD;
    SetVCore(2);
    msp430_clock_init(12000000L, 2);
    if (USB_init() != kUSB_succeed)
        msp430_reset();
    msp430_i2c_enable();
    msp430_int_init();

    USB_setEnabledEvents(kUSB_allUsbEvents);
    if (USB_connectionInfo() & kUSB_vbusPresent){
        if (USB_enable() == kUSB_succeed){
            USB_reset();
            USB_connect();
        } else
            msp430_reset();
    }


}
*/
int main(void)
{
    int result;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    unsigned long timestamp;
    struct int_param_s int_param;

    /* Set up Edison hardware. */
    //platform_init();
    //qq95538 This inline init function has no need. "int_param" or other
    //main function variables holding handles which actuate global variables need
    //initiate in the main loop would be not directly created by another function.
    /* Set up gyro.
     * Every function preceded by mpu_ is a driver function and can be found
     * in inv_mpu.h.
     */
    mraa_result_t mraa_result;
    mraa_result = mraa_init();
    if(mraa_result != MRAA_SUCCESS){
       	fprintf(stderr, "MRAA initiating failed.");
       	mraa_result_print(mraa_result);
    }
    i2c = mraa_i2c_init(6);
    if(i2c == NULL){
    	fprintf(stderr, "MRAA cannot initiate i2c bus 6.");
    }
    if(mraa_i2c_address(i2c, MPU6050_ADDRESS) != MRAA_SUCCESS){
    	fprintf(stderr, "MRAA cannot set i2c address.");
    }

    int_param.gpio_context_of_pin = mraa_gpio_init(INT_PIN);
    if (int_param.gpio_context_of_pin == NULL) {
    	fprintf(stderr, "MRAA cannot create an gpio for function reg_int_cb()");
    	return -1;
    }
    if( mraa_gpio_dir(int_param.gpio_context_of_pin, MRAA_GPIO_IN) != MRAA_SUCCESS){
    	fprintf(stderr, "MRAA setting gpio dir operation failed.");
    }
    int_param.cb = gyro_data_ready_cb;
    int_param.args = NULL;
    result = mpu_init(&int_param);
    if (result)
        edison_reset();

    /* If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);

    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON;
    hal.report = PRINT_QUAT;

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_orientation));
    dmp_register_tap_cb(tap_cb);
    //qq95538 android_orient_cb is not found.
    //dmp_register_android_orient_cb(android_orient_cb);
    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     */
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;

    //qq95538 Edison's version avoid interrupt and wake up mechanism in msp430 at the beginning.
    //__enable_interrupt();

    /* Wait for enumeration. */
    //while (USB_connectionState() != ST_ENUM_ACTIVE);

    //qq95538 Avoid multi-thread programming. Set hal.rx.cmd before main loop begins.
    //qq95538 Edison uses COM port scanf for command enumeration.

    while (1) {
        unsigned long sensor_timestamp;


        //qq95538 avoids multi-thread programming. move the code block out of main loop.
        //if (rx_new)
            /* A byte has been received via USB. See handle_input for a list of
             * valid commands.
             */
        //    handle_input();

        //msp430_get_clock_ms(&timestamp);
        edison_get_clock_ms(&timestamp);//qq95538 this function has no value on accessing DMP data process.

        //qq95538 avoid interrupt programming.
        //if (hal.motion_int_mode) {
            /* Enable motion interrupt. */
		//	mpu_lp_motion_interrupt(500, 1, 5);
        //    hal.new_gyro = 0;
            /* Wait for the MPU interrupt. */
        //    while (!hal.new_gyro)
        //        __bis_SR_register(LPM0_bits + GIE);
            /* Restore the previous sensor configuration. */
        //    mpu_lp_motion_interrupt(0, 0, 0);
        //    hal.motion_int_mode = 0;
        //}

        //if (!hal.sensors || !hal.new_gyro) {
            /* Put the MSP430 to sleep until a timer interrupt or data ready
             * interrupt is detected.
             */
        //    __bis_SR_register(LPM0_bits + GIE);
        //    continue;
        //}

        if (hal.new_gyro && hal.dmp_on) {
            short gyro[3], accel[3], sensors;
            unsigned char more;
            long quat[4];
            /* This function gets new data from the FIFO when the DMP is in
             * use. The FIFO can contain any combination of gyro, accel,
             * quaternion, and gesture data. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
             * the FIFO isn't being filled with accel data.
             * The driver parses the gesture data to determine if a gesture
             * event has occurred; on an event, the application will be notified
             * via a callback (assuming that a callback function was properly
             * registered). The more parameter is non-zero if there are
             * leftover packets in the FIFO.
             */
            dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
                &more);//qq95538 the KEY FUNCTION of this example.
            if (!more)
                hal.new_gyro = 0;
            /* Gyro and accel data are written to the FIFO by the DMP in chip
             * frame and hardware units. This behavior is convenient because it
             * keeps the gyro and accel outputs of dmp_read_fifo and
             * mpu_read_fifo consistent.
             */
            if ((sensors & INV_XYZ_GYRO) && (hal.report & PRINT_GYRO))
                //qq95538 avoid message transfer operation instead of print the data in the main loop directly.
            	//send_packet(PACKET_TYPE_GYRO, gyro);
            	printf("PACKET_TYPE_GYRO x:%d, y:%d, z:%d\n", gyro[0], gyro[1], gyro[2]);
            if ((sensors & INV_XYZ_ACCEL) && (hal.report & PRINT_ACCEL))
                //qq95538 avoid message transfer operation instead of print the data in the main loop directly.
                //send_packet(PACKET_TYPE_ACCEL, accel);
            	printf("PACKET_TYPE_ACCEL x:%d, y:%d, z:%d\n", accel[0], accel[1], accel[2]);
            /* Unlike gyro and accel, quaternions are written to the FIFO in
             * the body frame, q30. The orientation is set by the scalar passed
             * to dmp_set_orientation during initialization.
             */
            if ((sensors & INV_WXYZ_QUAT) && (hal.report & PRINT_QUAT))
                //qq95538 avoid message transfer operation instead of print the data in the main loop directly.
            	//send_packet(PACKET_TYPE_QUAT, quat);
            	printf("PACKET_TYPE_QUAT q0:%ld, q1:%ld, q2:%ld, q3:%ld\n", quat[0],quat[1], quat[2], quat[4]);
        } else if (hal.new_gyro) {
            short gyro[3], accel[3];
            unsigned char sensors, more;
            /* This function gets new data from the FIFO. The FIFO can contain
             * gyro, accel, both, or neither. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
             * being filled with accel data. The more parameter is non-zero if
             * there are leftover packets in the FIFO.
             */
            mpu_read_fifo(gyro, accel, &sensor_timestamp, &sensors, &more);
            if (!more)
                hal.new_gyro = 0;
            if ((sensors & INV_XYZ_GYRO) && (hal.report & PRINT_GYRO))
                //qq95538 avoid message transfer operation instead of print the data in the main loop directly.
            	//send_packet(PACKET_TYPE_GYRO, gyro);
            	printf("PACKET_TYPE_GYRO x:%d, y:%d, z:%d\n", gyro[0], gyro[1], gyro[2]);
            if ((sensors & INV_XYZ_ACCEL) && (hal.report & PRINT_ACCEL))
            	//qq95538 avoid message transfer operation instead of print the data in the main loop directly.
                //send_packet(PACKET_TYPE_ACCEL, accel);
            	printf("PACKET_TYPE_ACCEL x:%d, y:%d, z:%d\n", accel[0], accel[1], accel[2]);
        }

    }
}

