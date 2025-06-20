CXX       := g++
CXXFLAGS  := -Wall -Wextra -std=c++17 -pthread -MMD -MP \
             -I. \
             -I./radar_sensor \
             -I./radar_sensor/sensor_data \
             -I./radar_sensor/sensor_data/lib

SRCS      := Radar.cpp \
             radar_sensor/IWR6843.cpp \
             radar_sensor/SensorData.cpp \
             radar_sensor/sensor_data/UARTFrame.cpp \
             radar_sensor/sensor_data/lib/EndianUtils.cpp

OBJS      := $(SRCS:.cpp=.o)
DEPS      := $(SRCS:.cpp=.d)
TARGET    := radar_app

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

-include $(DEPS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(DEPS) $(TARGET)

.PHONY: all clean
