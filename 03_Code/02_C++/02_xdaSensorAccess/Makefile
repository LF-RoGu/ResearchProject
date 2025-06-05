CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17
LDFLAGS = -ludev
TARGET = mti_decode
SRC = mti_decode.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET)
