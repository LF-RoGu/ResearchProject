# Makefile

# C++ compiler and flags
CXX       := g++
CXXFLAGS  := -std=c++17 -Wall -Wextra -pthread

# C compiler and flags
CC        := gcc
CFLAGS    := -std=c11 -Wall -Wextra

# Include paths
INCLUDES  := -I. -ImmWave-IWR6843 -IMTi-G-710

# Sources
SRCS_CPP := \
  main.cpp \
  mmWave-IWR6843/radar_sensor/IWR6843.cpp \
  mmWave-IWR6843/radar_sensor/SensorData.cpp \
  mmWave-IWR6843/radar_sensor/sensor_data/UARTFrame.cpp \
  mmWave-IWR6843/radar_sensor/sensor_data/lib/EndianUtils.cpp \
  MTi-G-710/xsens_mti710.cpp

SRCS_C := \
  MTi-G-710/libs/xsens_mti.c \
  MTi-G-710/libs/xsens_utility.c \
  MTi-G-710/libs/xsens_mdata2.c

# Object files
OBJS_CPP := $(SRCS_CPP:.cpp=.o)
OBJS_C   := $(SRCS_C:.c=.o)
OBJS     := $(OBJS_CPP) $(OBJS_C)

TARGET := main

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $(OBJS) -ludev

# C++ compilation rule
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# C compilation rule
%.o: %.c
	$(CC)  $(CFLAGS)  $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
