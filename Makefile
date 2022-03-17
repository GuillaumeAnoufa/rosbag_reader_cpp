# This Makefile is to test the library libhpad_detector.so
CFLAGS = -fPIC -Wall -std=c++14 -Wno-deprecated-declarations -Wno-unused-variable
ifeq ($(DEBUG), 1)
	CFLAGS += -ggdb -Og
else
	CFLAGS += -O2
endif
CXX = g++ $(CFLAGS)

# PATHS
HPAD_DETECTOR_PATH = /home/ganoufa/workSpace/camera/helipad_detector
ROS_PATH = /opt/ros/noetic
OBJ_PATH = build

# LIBRARIES
OPENCV_LIBS = -lopencv_highgui -lopencv_imgproc -lopencv_core -lopencv_videoio \
-lopencv_calib3d -lopencv_imgcodecs

ROS_LIBS = -lrosbag -lrosbag_storage -lroscpp -lcpp_common -lroslib -lrostime -lroscpp_serialization

LIBS = $(OPENCV_LIBS) $(ROS_LIBS) -ltbb -lpthread -lglog -lhpad_detector

# COMPILE FLAGS
LFLAGS 		= -L$(CUDA_BIN_PATH)/lib64 -L$(OPENCV_DIR)/lib -L$(ROS_PATH)/lib
CUDA_FLAGS = -lcuda -lcudart -lopencv_cudaarithm

# INCLUDES
OPENCV_INC  = -I$(OPENCV_DIR)/include/opencv4
CUDA_INC = -I$(CUDA_BIN_PATH)/include
ROS_INC = -I$(ROS_PATH)/include/

ELLIPSE_INC = -I$(HPAD_DETECTOR_PATH)/src -I$(HPAD_DETECTOR_PATH)/src/tools \
-I$(HPAD_DETECTOR_PATH)/src/tangent -I$(HPAD_DETECTOR_PATH)/src/cudaMemory \
-I$(HPAD_DETECTOR_PATH)/src/process -I$(HPAD_DETECTOR_PATH)/src/connectedComponents \
-I$(HPAD_DETECTOR_PATH)/src/hough -I$(HPAD_DETECTOR_PATH)/src/ellipse \
-I$(HPAD_DETECTOR_PATH)/src/tracking -I$(HPAD_DETECTOR_PATH)/src/validation \
-I$(HPAD_DETECTOR_PATH)/src/angles -I$(HPAD_DETECTOR_PATH)/src/subImage

INCLUDE = $(OPENCV_INC) $(ELLIPSE_INC) $(CUDA_INC) $(ROS_INC)

# APPS
all: build/main

build/main: $(OBJ_PATH)/main.o $(ELLIPSE_OBJECTS)
	$(CXX) -o $@ $^ $(LFLAGS) $(LIBS) $(CUDA_FLAGS)

build/main.o: main.cpp
	$(CXX) -o $@ -c $< $(INCLUDE)
clean:
	find . -name "*.log" -type f -delete
	rm -f build/*
