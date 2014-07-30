SRC = ARC_IMU.cpp ARC_Pair.cpp ARC_Sensors.cpp ReflectionTracker.cpp
OBJ = $(SRC:.cpp=.o)
IDIR = ../../../slam/include

CXXFLAGS = -Wall -Werror 
CXXFLAGS += -g
CXXFLAGS += $(shell pkg-config --cflags opencv)
CXXFLAGS += -I$(IDIR)
LDIR = ../../../slam/lib
LIBS += $(LDIR)/libARC.a
LIBS += $(shell pkg-config --libs opencv)

segment: $(OBJ) main.o
	$(CXX) -o ../bin/segment $(OBJ) main.o $(CXXFLAGS) $(LIBS) 

.cpp.o:
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -f *.o
	rm -f ../bin/segment

