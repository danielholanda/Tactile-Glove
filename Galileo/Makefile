CC = gcc
CPP = g++
CXXFLAGS = -DDMP_FIFO_RATE=9 -Wall -g -O2

all: I2Cdev.o MPU6050.o readOneIMU readTwoIMUs testIO tactileHand

I2Cdev.o: I2Cdev.cpp
	$(CPP) $(CXXFLAGS) -c -o I2Cdev.o I2Cdev.cpp

MPU6050.o: MPU6050.cpp
	$(CPP) $(CXXFLAGS) -c -o MPU6050.o MPU6050.cpp

readOneIMU: I2Cdev.o MPU6050.o readOneIMU.cpp
	$(CPP)  readOneIMU.cpp -o readOneIMU -lm I2Cdev.o MPU6050.o

readTwoIMUs: I2Cdev.o MPU6050.o readTwoIMUs.cpp
	$(CPP)  readTwoIMUs.cpp -o readTwoIMUs -lm I2Cdev.o MPU6050.o

testIO: testIO.c
	$(CC) testIO.c -o testIO -lmraa

tactileHand: I2Cdev.o MPU6050.o tactileHand.cpp
	$(CPP)  tactileHand.cpp -o tactileHand -lm -lmraa I2Cdev.o MPU6050.o
