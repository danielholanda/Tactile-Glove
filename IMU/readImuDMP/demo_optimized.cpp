#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu_head;
MPU6050 mpu_body;

//#define OUTPUT_READABLE_QUATERNION
#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL
#define OUTPUT_READABLE_WORLDACCEL
//#define OUTPUT_TEAPOT

// MPU control/status vars
bool dmpHeadReady = false;  // set true if DMP init was successful
bool dmpBodyReady = false;  // set true if DMP init was successful

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    dmpHeadReady = mpu_head.dmpStartDevice(0x68, 0, 0, 0);
    
    dmpBodyReady = mpu_body.dmpStartDevice(0x69, 0, 0, 0);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    if(mpu_head.dmpGetData())
    {
        /*
        //printf("quat %7.2f %7.2f %7.2f %7.2f    ", dmpQuat.w,dmpQuat.x,dmpQuat.y,dmpQuat.z);
        //printf("accell %6d %6d %6d    ", dmpAccel.x, dmpAccel.y, dmpAccel.z);
        //printf("areal %6d %6d %6d    ", dmpAccelReal.x, dmpAccelReal.y, dmpAccelReal.z);
        printf("aworld %6d %6d %6d    ", dmpAccelWorld.x, dmpAccelWorld.y, dmpAccelWorld.z);
        //printf("gravity %7.2f %7.2f %7.2f    ", dmpGravity.x, dmpGravity.y, dmpGravity.z);
        printf("euler %7.2f %7.2f %7.2f    ", dmpEuler[0], dmpEuler[1], dmpEuler[2]);
        printf("ypr  %7.2f %7.2f %7.2f    ", dmpYawPitchRoll[0], dmpYawPitchRoll[1], dmpYawPitchRoll[2]);
        printf("\n");
         */
    }
    
    if(mpu_body.dmpGetData())
    {
        /*
        //printf("quat %7.2f %7.2f %7.2f %7.2f    ", dmpQuat.w,dmpQuat.x,dmpQuat.y,dmpQuat.z);
        //printf("accell %6d %6d %6d    ", dmpAccel.x, dmpAccel.y, dmpAccel.z);
        //printf("areal %6d %6d %6d    ", dmpAccelReal.x, dmpAccelReal.y, dmpAccelReal.z);
        printf("aworld %6d %6d %6d    ", dmpAccelWorld.x, dmpAccelWorld.y, dmpAccelWorld.z);
        //printf("gravity %7.2f %7.2f %7.2f    ", dmpGravity.x, dmpGravity.y, dmpGravity.z);
        printf("euler %7.2f %7.2f %7.2f    ", dmpEuler[0], dmpEuler[1], dmpEuler[2]);
        printf("ypr  %7.2f %7.2f %7.2f    ", dmpYawPitchRoll[0], dmpYawPitchRoll[1], dmpYawPitchRoll[2]); 
        printf("\n");
         * */
    }
}

int main() {
    setup();
    usleep(100000);
    
    for (;;)
        loop();
    
    return 0;
}

