# Makefile

CC       := gcc
CXX      := g++
CFLAGS   := -Wall -Wextra -std=c11   -MMD -MP
CXXFLAGS := -Wall -Wextra -std=c++17 -MMD -MP
LDFLAGS  := -ludev -lm

TARGET    := mti_request

# 1) your C++ sources
SRCS      := mti_request.cpp

# 2) your C libraries
CSRCS     := \
    libs/xsens_mti.c \
    libs/xsens_utility.c \
    libs/xsens_mdata2.c

OBJS      := $(SRCS:.cpp=.o) $(CSRCS:.c=.o)
DEPS      := $(SRCS:.cpp=.d) $(CSRCS:.c=.d)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $@ $(LDFLAGS)

# C++ compilation
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# C compilation
%.o: %.c
	$(CC)  $(CFLAGS)  -c $< -o $@

-include $(DEPS)

clean:
	rm -f $(TARGET) $(OBJS) $(DEPS)
