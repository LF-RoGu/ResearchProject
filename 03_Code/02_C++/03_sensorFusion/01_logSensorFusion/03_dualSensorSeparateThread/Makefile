# Makefile

# C++ compiler and flags
CXX       := g++
CXXFLAGS  := -std=c++17 -Wall -Wextra -pthread

# C compiler and flags
CC        := gcc
CFLAGS    := -std=c11 -Wall -Wextra

# Include paths
INCLUDES  := -I. -ImmWave-IWR6843 -IMTi-G-710 -Imisc

# Output directory
OUT_DIR   := _out

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

# Convert source files to object files in _out directory
OBJS_CPP := $(SRCS_CPP:.cpp=.o)
OBJS_C   := $(SRCS_C:.c=.o)
OBJS     := $(OBJS_CPP) $(OBJS_C)
OBJS     := $(addprefix $(OUT_DIR)/, $(OBJS))

TARGET := main

.PHONY: all clean

all: $(OUT_DIR) $(TARGET)

$(OUT_DIR):
	mkdir -p $(OUT_DIR)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $(OBJS) -ludev

# C++ compilation rule
$(OUT_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# C compilation rule
$(OUT_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC)  $(CFLAGS)  $(INCLUDES) -c $< -o $@

clean:
	rm -rf $(OUT_DIR) $(TARGET)
