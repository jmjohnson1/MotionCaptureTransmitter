# Makefile

BINARY=bin

IDIR = ./include
CXX := g++
CXXFLAGS = -fPIC -Wall -g -O3 -funroll-loops -I$(IDIR)
LDFLAGS = -v -Wl,-rpath ./ -L.
LDLIBS = -lrt -lowlsock -lpthread
LINK.o = $(LINK.cc)
DEPS = mavlink.h generic_port.h serial_port.h udp_port.h
CXXFILES = transmitDataTest.cpp serial_port.cpp udp_port.cpp
OBJECTS = transmitDataTest.o serial_port.o  udp_port.o


all: $(BINARY)

$(BINARY): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $^


clean:
	rm -rf $(BINARY) $(OBJECTS)
