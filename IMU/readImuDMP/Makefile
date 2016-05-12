
objects=demo_raw demo_dmp demo_optimized
all: clean $(objects)

HDRS = helper_3dmath.h I2Cdev.h MPU6050_6Axis_MotionApps20.h MPU6050.h
CMN_OBJS = I2Cdev.o MPU6050.o
RAW_OBJS = demo_raw.o
DMP_OBJS = demo_dmp.o
OPT_OBJS = demo_optimized.o

CXXFLAGS = -DDMP_FIFO_RATE=9 -Wall -g -O2

$(CMN_OBJS) $(DMP_OBJS) : $(HDRS)

demo_raw: $(CMN_OBJS) $(RAW_OBJS)
	$(CXX) -o $@ $^ -lm

demo_dmp: $(CMN_OBJS) $(DMP_OBJS)
	$(CXX) -o $@ $^ -lm
	
demo_optimized: $(CMN_OBJS) $(OPT_OBJS)
	$(CXX) -o $@ $^ -lm
	
.PHONY:clean
clean:
	-rm -f $(objects) *.o
	echo Clean done
    