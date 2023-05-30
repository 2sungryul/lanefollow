CX = g++
CXFLAGS = -g -Wall 

DXLFLAGS = -I/usr/local/include/dynamixel_sdk_cpp
DXLFLAGS += -ldxl_x64_cpp              #/usr/local/lib/libdxl_x64_cpp.so               
DXLFLAGS += -lrt

CVFLAGS = `pkg-config opencv4 --cflags --libs`

BUILDFLAGS = $(DXLFLAGS)
BUILDFLAGS += $(CVFLAGS)

TARGET = lanefollow
OBJS = main.o vision.o dxl.o
$(TARGET) :  $(OBJS)
	$(CX) $(CXFLAGS) -o $(TARGET) $(OBJS) $(BUILDFLAGS) 
main.o : main.cpp
	$(CX) $(CXFLAGS) -c main.cpp $(BUILDFLAGS) 
vision.o : vision.hpp vision.cpp
	$(CX) $(CXFLAGS) -c vision.cpp $(CVFLAGS) 
dxl.o : dxl.hpp dxl.cpp
	$(CX) $(CXFLAGS) -c dxl.cpp $(DXLFLAGS)

.PHONY: all clean
all: $(TARGET)

clean:
	rm -rf $(TARGET) $(OBJS)


