CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17 -MMD -MP
LDFLAGS = -ludev -lm
TARGET = mti_request

SRC = mti_request.cpp
XSENS_LIBS = \
    libs/xsens_mti.c \
    libs/xsens_utility.c \
    libs/xsens_mdata2.c

OBJ = $(SRC:.cpp=.o) $(XSENS_LIBS:.c=.o)
DEP = $(SRC:.cpp=.d)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

-include $(DEP)

clean:
	rm -f $(TARGET) $(OBJ) $(DEP)
