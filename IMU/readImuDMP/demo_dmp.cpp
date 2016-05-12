#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu_head(0x68);
MPU6050 mpu_body(0x69);

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// MPU control/status vars
bool dmpHeadReady = false;  // set true if DMP init was successful
bool dmpBodyReady = false;  // set true if DMP init was successful
uint8_t mpu_headIntStatus;   // holds actual interrupt status byte from MPU
uint8_t mpu_bodyIntStatus;   // holds actual interrupt status byte from MPU
uint8_t mpu_headStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t mpu_bodyStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t headPacketSize;    // expected DMP packet size (default is 42 bytes)
uint16_t bodyPacketSize;    // expected DMP packet size (default is 42 bytes)
uint16_t headFifoCount;     // count of all bytes currently in FIFO
uint8_t headFifoBuffer[64]; // FIFO storage buffer
uint16_t bodyFifoCount;     // count of all bytes currently in FIFO
uint8_t bodyFifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // initialize device
    printf("Initializing I2C devices...\n");
    mpu_head.initialize();
    mpu_body.initialize();

    // verify connection
    printf("Testing device connections...\n");
    printf(mpu_head.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
    printf(mpu_body.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
    
    // load and configure the DMP
    printf("Initializing DMP...\n");
    mpu_headStatus = mpu_head.dmpInitialize();
    mpu_bodyStatus = mpu_body.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (mpu_headStatus == 0 && mpu_bodyStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu_head.setDMPEnabled(true);
        mpu_body.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpu_headIntStatus = mpu_head.getIntStatus();
        mpu_bodyIntStatus = mpu_head.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpHeadReady = true;
        dmpBodyReady = true;

        // get expected DMP packet size for later comparison
        headPacketSize = mpu_head.dmpGetFIFOPacketSize();
        bodyPacketSize = mpu_body.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", mpu_headStatus);
        printf("DMP Initialization failed (code %d)\n", mpu_bodyStatus);
        
    }
    
    
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (dmpHeadReady)
    {
        // get current FIFO count
        headFifoCount = mpu_head.getFIFOCount();

        if (headFifoCount == 1024) {
            // reset so we can continue cleanly
            mpu_head.resetFIFO();
            printf("FIFO overflow!\n");

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (headFifoCount >= 42) {
            // read a packet from FIFO
            mpu_head.getFIFOBytes(headFifoBuffer, headPacketSize);

            #ifdef OUTPUT_READABLE_QUATERNION
                // display quaternion values in easy matrix form: w x y z
                mpu_head.dmpGetQuaternion(&q, headFifoBuffer);
                printf("quat %7.2f %7.2f %7.2f %7.2f    ", q.w,q.x,q.y,q.z);
            #endif

            #ifdef OUTPUT_READABLE_EULER
                // display Euler angles in degrees
                mpu_head.dmpGetQuaternion(&q, headFifoBuffer);
                mpu_head.dmpGetEuler(euler, &q);
                printf("euler %7.2f %7.2f %7.2f    ", euler[0] * 180/M_PI, euler[1] * 180/M_PI, euler[2] * 180/M_PI);
            #endif

            #ifdef OUTPUT_READABLE_YAWPITCHROLL
                // display Euler angles in degrees
                mpu_head.dmpGetQuaternion(&q, headFifoBuffer);
                mpu_head.dmpGetGravity(&gravity, &q);
                mpu_head.dmpGetYawPitchRoll(ypr, &q, &gravity);
                printf("ypr  %7.2f %7.2f %7.2f    ", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
            #endif

            #ifdef OUTPUT_READABLE_REALACCEL
                // display real acceleration, adjusted to remove gravity
                mpu_head.dmpGetQuaternion(&q, headFifoBuffer);
                mpu_head.dmpGetAccel(&aa, headFifoBuffer);
                mpu_head.dmpGetGravity(&gravity, &q);
                mpu_head.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                printf("areal %6d %6d %6d    ", aaReal.x, aaReal.y, aaReal.z);
            #endif

            #ifdef OUTPUT_READABLE_WORLDACCEL
                // display initial world-frame acceleration, adjusted to remove gravity
                // and rotated based on known orientation from quaternion
                mpu_head.dmpGetQuaternion(&q, headFifoBuffer);
                mpu_head.dmpGetAccel(&aa, headFifoBuffer);
                mpu_head.dmpGetGravity(&gravity, &q);
                mpu_head.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
                printf("aworld %6d %6d %6d    ", aaWorld.x, aaWorld.y, aaWorld.z);
            #endif

            #ifdef OUTPUT_TEAPOT
                // display quaternion values in InvenSense Teapot demo format:
                teapotPacket[2] = headFifoBuffer[0];
                teapotPacket[3] = headFifoBuffer[1];
                teapotPacket[4] = headFifoBuffer[4];
                teapotPacket[5] = headFifoBuffer[5];
                teapotPacket[6] = headFifoBuffer[8];
                teapotPacket[7] = headFifoBuffer[9];
                teapotPacket[8] = headFifoBuffer[12];
                teapotPacket[9] = headFifoBuffer[13];
                Serial.write(teapotPacket, 14);
                teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
            #endif
            printf("\n");
        }
    }
    
    
    if (dmpBodyReady)
    {
        // get current FIFO count
        bodyFifoCount = mpu_body.getFIFOCount();

        if (bodyFifoCount == 1024) {
            // reset so we can continue cleanly
            mpu_body.resetFIFO();
            printf("FIFO overflow!\n");

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (bodyFifoCount >= 42) {
            // read a packet from FIFO
            mpu_body.getFIFOBytes(bodyFifoBuffer, bodyPacketSize);

            #ifdef OUTPUT_READABLE_QUATERNION
                // display quaternion values in easy matrix form: w x y z
                mpu_body.dmpGetQuaternion(&q, bodyFifoBuffer);
                printf("quat %7.2f %7.2f %7.2f %7.2f    ", q.w,q.x,q.y,q.z);
            #endif

            #ifdef OUTPUT_READABLE_EULER
                // display Euler angles in degrees
                mpu_body.dmpGetQuaternion(&q, bodyFifoBuffer);
                mpu_body.dmpGetEuler(euler, &q);
                printf("euler %7.2f %7.2f %7.2f    ", euler[0] * 180/M_PI, euler[1] * 180/M_PI, euler[2] * 180/M_PI);
            #endif

            #ifdef OUTPUT_READABLE_YAWPITCHROLL
                // display Euler angles in degrees
                mpu_body.dmpGetQuaternion(&q, bodyFifoBuffer);
                mpu_body.dmpGetGravity(&gravity, &q);
                mpu_body.dmpGetYawPitchRoll(ypr, &q, &gravity);
                printf("ypr  %7.2f %7.2f %7.2f    ", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
            #endif

            #ifdef OUTPUT_READABLE_REALACCEL
                // display real acceleration, adjusted to remove gravity
                mpu_body.dmpGetQuaternion(&q, bodyFifoBuffer);
                mpu_body.dmpGetAccel(&aa, bodyFifoBuffer);
                mpu_body.dmpGetGravity(&gravity, &q);
                mpu_body.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                printf("areal %6d %6d %6d    ", aaReal.x, aaReal.y, aaReal.z);
            #endif

            #ifdef OUTPUT_READABLE_WORLDACCEL
                // display initial world-frame acceleration, adjusted to remove gravity
                // and rotated based on known orientation from quaternion
                mpu_body.dmpGetQuaternion(&q, bodyFifoBuffer);
                mpu_body.dmpGetAccel(&aa, bodyFifoBuffer);
                mpu_body.dmpGetGravity(&gravity, &q);
                mpu_body.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
                printf("aworld %6d %6d %6d    ", aaWorld.x, aaWorld.y, aaWorld.z);
            #endif

            #ifdef OUTPUT_TEAPOT
                // display quaternion values in InvenSense Teapot demo format:
                teapotPacket[2] = bodyFifoBuffer[0];
                teapotPacket[3] = bodyFifoBuffer[1];
                teapotPacket[4] = bodyFifoBuffer[4];
                teapotPacket[5] = bodyFifoBuffer[5];
                teapotPacket[6] = bodyFifoBuffer[8];
                teapotPacket[7] = bodyFifoBuffer[9];
                teapotPacket[8] = bodyFifoBuffer[12];
                teapotPacket[9] = bodyFifoBuffer[13];
                Serial.write(teapotPacket, 14);
                teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
            #endif
            printf("\n");
        }
    }
    
}

int main() {
    setup();
    usleep(100000);
    
    for (;;)
        loop();
    
    return 0;
}

