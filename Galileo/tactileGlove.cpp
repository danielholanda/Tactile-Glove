//All includes
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
#include <errno.h>
#include <signal.h>
#include "mraa.h"

//Defining IO outputs of each finger
#define thumb_f 8
#define index_f 9
#define middle_f 10
#define ring_f 11
#define pinky_f 12

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu_Arm(0x68);
MPU6050 mpu_Hand(0x69);



// MPU control/status vars
bool dmpArmReady = false;  		// set true if DMP init was successful
bool dmpHandReady = false;  	
uint8_t mpu_ArmIntStatus;   	// holds actual interrupt status byte from MPU
uint8_t mpu_HandIntStatus;   	
uint8_t mpu_ArmStatus;      	// return status after each device operation (0 = success, !0 = error)
uint8_t mpu_HandStatus;      	
uint16_t ArmPacketSize;    		// expected DMP packet size (default is 42 bytes)
uint16_t HandPacketSize;    	
uint16_t ArmFifoCount;     		// count of all bytes currently in FIFO
uint16_t HandFifoCount;     	
uint8_t ArmFifoBuffer[64]; 		// FIFO storage buffer
uint8_t HandFifoBuffer[64]; 	

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Packet structure 
uint8_t quaternionPacket[20] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, '\r', '\n' };

//Flags to indicate that data is ready
bool imu1IsOK;
bool imu2IsOK;

//Prepares Socket connection
int clientSocket;
struct sockaddr_in serverAddr;
socklen_t addr_size;

//For GPIO 
mraa_gpio_context gpio[5];


void setup() {


	//Socket configuration
	clientSocket = socket(PF_INET, SOCK_STREAM, 0);
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(7891);
	serverAddr.sin_addr.s_addr = inet_addr("10.0.0.106");
	memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);

    //Connect the socket to the server using the address struct
	addr_size = sizeof serverAddr;
	printf("Trying to connect to virtual environment...");
	connect(clientSocket, (struct sockaddr *) &serverAddr, addr_size);


    // Variables for checking if data is ready
	imu1IsOK=false;
	imu2IsOK=false;

    // initialize device
	printf("Initializing I2C devices...\n");
	mpu_Arm.initialize();
	mpu_Hand.initialize();

    // verify connection
	printf("Testing device connections...\n");
	printf(mpu_Arm.testConnection() ? "MPU6050 arm connection successful\n" : "MPU6050 arm connection failed\n");
	printf(mpu_Hand.testConnection() ? "MPU6050 hand connection successful\n" : "MPU6050 hand connection failed\n");

    // load and configure the DMP
	printf("Initializing DMP...\n");
	mpu_ArmStatus = mpu_Arm.dmpInitialize();
	mpu_HandStatus = mpu_Hand.dmpInitialize();

    // make sure it worked (returns 0 if so)
	if (mpu_ArmStatus == 0 && mpu_HandStatus == 0) {
        // turn on the DMP, now that it's ready
		printf("Enabling DMP...\n");
		mpu_Arm.setDMPEnabled(true);
		mpu_Hand.setDMPEnabled(true);

        // Get Interruption Status
		mpu_ArmIntStatus = mpu_Arm.getIntStatus();
		mpu_HandIntStatus = mpu_Arm.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
		printf("DMP ready!\n");
		dmpArmReady = true;
		dmpHandReady = true;

        // get expected DMP packet size for later comparison
		ArmPacketSize = mpu_Arm.dmpGetFIFOPacketSize();
		HandPacketSize = mpu_Hand.dmpGetFIFOPacketSize();
	} else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
		printf("DMP Arm Initialization failed (code %d)\n", mpu_ArmStatus);
		printf("DMP Hand Initialization failed (code %d)\n", mpu_HandStatus);

	}

    // Setting GPIO (Order is changed due to the way fingers are drawed on Processing)
	mraa_init();
	fprintf(stdout, "MRAA Version: %s\n", mraa_get_version());
	gpio[0] = mraa_gpio_init(thumb_f);
	gpio[4] = mraa_gpio_init(index_f);
	gpio[3] = mraa_gpio_init(middle_f);
	gpio[2] = mraa_gpio_init(ring_f);
	gpio[1] = mraa_gpio_init(pinky_f);

    // Set GPIO direction to OUT
	mraa_gpio_dir(gpio[0], MRAA_GPIO_OUT);
	mraa_gpio_dir(gpio[1], MRAA_GPIO_OUT);
	mraa_gpio_dir(gpio[2], MRAA_GPIO_OUT);
	mraa_gpio_dir(gpio[3], MRAA_GPIO_OUT);
	mraa_gpio_dir(gpio[4], MRAA_GPIO_OUT);

}



void loop() {
    
	//Reads IMU data from Arm
	if (dmpArmReady)
	{
        // Get current FIFO count
		ArmFifoCount = mpu_Arm.getFIFOCount();

		if (ArmFifoCount == 1024) {
            // Reset so we can continue cleanly
			mpu_Arm.resetFIFO();
			printf("FIFO overflow!\n");

        // Otherwise, check for DMP data ready interrupt (this should happen frequently)
		} else if (ArmFifoCount >= 42) {
            // Read a packet from FIFO
			mpu_Arm.getFIFOBytes(ArmFifoBuffer, ArmPacketSize);

            // Create Packet
			quaternionPacket[2] = ArmFifoBuffer[0];
			quaternionPacket[3] = ArmFifoBuffer[1];
			quaternionPacket[4] = ArmFifoBuffer[4];
			quaternionPacket[5] = ArmFifoBuffer[5];
			quaternionPacket[6] = ArmFifoBuffer[8];
			quaternionPacket[7] = ArmFifoBuffer[9];
			quaternionPacket[8] = ArmFifoBuffer[12];
			quaternionPacket[9] = ArmFifoBuffer[13];
			imu1IsOK=true;

		}
	}

	//Reads IMU data from Hand
	if (dmpHandReady)
	{
        // Get current FIFO count
		HandFifoCount = mpu_Hand.getFIFOCount();

		if (HandFifoCount == 1024) {
            // Reset so we can continue cleanly
			mpu_Hand.resetFIFO();
			printf("FIFO overflow!\n");

        // Otherwise, check for DMP data ready interrupt (this should happen frequently)
		} else if (HandFifoCount >= 42) {
            // Read a packet from FIFO
			mpu_Hand.getFIFOBytes(HandFifoBuffer, HandPacketSize);

            // Create Packet
			quaternionPacket[10] = HandFifoBuffer[0];
			quaternionPacket[11] = HandFifoBuffer[1];
			quaternionPacket[12] = HandFifoBuffer[4];
			quaternionPacket[13] = HandFifoBuffer[5];
			quaternionPacket[14] = HandFifoBuffer[8];
			quaternionPacket[15] = HandFifoBuffer[9];
			quaternionPacket[16] = HandFifoBuffer[12];
			quaternionPacket[17] = HandFifoBuffer[13];
			imu2IsOK=true;

			printf("\n");
		}
	}

	//Data is OK if I can read from both IMUs
	if(imu1IsOK && imu2IsOK){

		//Set flags to false
		imu1IsOK=false;
		imu2IsOK=false;

		printf("DataIsOk\n");

		//Send message through socket
		send(clientSocket,quaternionPacket,sizeof(quaternionPacket),0);

		//Read incoming message saying fingers to vibrate
		char buffer_recv[1];
		recv(clientSocket, buffer_recv, sizeof(buffer_recv), 0);
		uint8_t finger = (uint8_t)buffer_recv[0];

		//Vibrate fingers according to message received
		for(int i=0;i<5;i++){
			if(finger&(1<<i)){
				mraa_gpio_write(gpio[i],1);

			}else{
				mraa_gpio_write(gpio[i],0);
			}
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

