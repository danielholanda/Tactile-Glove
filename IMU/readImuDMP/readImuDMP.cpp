#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

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

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
#define OUTPUT_TEAPOT

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
uint8_t teapotPacket[20] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, '\r', '\n' };

//Checks is data is ready to send
bool imu1IsOK;
bool imu2IsOK;

//Prepares SOcket connection
int clientSocket;
struct sockaddr_in serverAddr;
socklen_t addr_size;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {



    clientSocket = socket(PF_INET, SOCK_STREAM, 0);

    /*---- Configure settings of the server address struct ----*/
    /* Address family = Internet */
    serverAddr.sin_family = AF_INET;
    /* Set port number, using htons function to use proper byte order */
    serverAddr.sin_port = htons(7891);
    /* Set IP address to localhost */
    serverAddr.sin_addr.s_addr = inet_addr("10.13.100.57");
    /* Set all bits of the padding field to 0 */
    memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);

    /*---- Connect the socket to the server using the address struct ----*/
    addr_size = sizeof serverAddr;
    connect(clientSocket, (struct sockaddr *) &serverAddr, addr_size);


    // Variables for checking if data is ready
    imu1IsOK=false;
    imu2IsOK=false;
  
    // initialize device
    printf("Initializing I2C devices...\n");
    mpu_head.initialize();
   //usleep(100);
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
                printf("IMU1: quat %7.2f %7.2f %7.2f %7.2f    ", q.w,q.x,q.y,q.z);
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
                //teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
		imu1IsOK=true;
            #endif
            //printf("\n");
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
                printf("IMU2: quat %7.2f %7.2f %7.2f %7.2f    ", q.w,q.x,q.y,q.z);
            #endif

            #ifdef OUTPUT_TEAPOT
                // display quaternion values in InvenSense Teapot demo format:
                teapotPacket[10] = bodyFifoBuffer[0];
                teapotPacket[11] = bodyFifoBuffer[1];
                teapotPacket[12] = bodyFifoBuffer[4];
                teapotPacket[13] = bodyFifoBuffer[5];
                teapotPacket[14] = bodyFifoBuffer[8];
                teapotPacket[15] = bodyFifoBuffer[9];
                teapotPacket[16] = bodyFifoBuffer[12];
                teapotPacket[17] = bodyFifoBuffer[13];
                //teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
		imu2IsOK=true;
            #endif
            printf("\n");
        }
    }

    if(imu1IsOK && imu2IsOK){
	    //Serial.write(teapotPacket, 20);
	    imu1IsOK=false;
	    imu2IsOK=false;
    
    	printf("DataIsOk\n");


   	send(clientSocket,teapotPacket,sizeof(teapotPacket),0);
	
    

    char buffer_recv[1];
    recv(clientSocket, buffer_recv, sizeof(buffer_recv), 0);
          printf("Data received: %s\n",buffer_recv);
    }
}

int main() {
    setup();
    usleep(100000);
    
    for (;;)
        loop();
    
    return 0;
}

