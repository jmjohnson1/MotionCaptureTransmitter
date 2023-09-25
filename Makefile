# Makefile

IDIR = ./include
CXX := g++
CXXFLAGS = -fPIC -Wall -g -O3 -funroll-loops -I$(IDIR)
LDFLAGS = -Wl,-rpath ./ -L.
LDLIBS = -lrt -lowlsock
LINK.o = $(LINK.cc)
DEPS = mavlink.h

TARGETS := logDataTest transmitDataTest

all: $(TARGETS)

clean:
	rm -f *.o $(TARGETS)
