CC=c++
CFLAGS=-std=c++11 -g -Wfatal-errors -Wall
LDFLAGS=../externals/isam_v1_7/lib/libisam.a -ladolc -L../build/lib64 \
	-lcholmod -lbot2-lcmgl-client -llcm -L../build/lib
INCFLAGS=-I../build/include \
	-I./include \
	-I../externals/isam_v1_7/include \
	-I/usr/include/eigen3 \

all: src/isamSP.cpp
	$(CC) $(CFLAGS) src/isamSP.cpp $(LDFLAGS) $(INCFLAGS) -o bin/isamSP

test: src/test.cpp
	$(CC) $(CFLAGS) src/test.cpp $(LDFLAGS) $(INCFLAGS) -o bin/test
	
adoltest: src/tapeless_scalar.cpp
	$(CC) $(CFLAGS) src/tapeless_scalar.cpp -ladolc -L../build/lib64  -I../build/include -o bin/tapeless_scalar 

cppadtest: src/get_started.cpp build
	$(CC) $(CFLAGS) src/get_started.cpp  -I$(CODE_BASE)/build/include -o bin/get_started 

cppadeigentest: src/eigen_det.cpp build
	$(CC) $(CFLAGS) src/eigen_det.cpp  -I$(CODE_BASE)/build/include -I/usr/include/eigen3 -o bin/eigen_det
	
cppadarraytest: build
	$(CC) $(CFLAGS) src/eigen_array.cpp  -I$(CODE_BASE)/build/include -I/usr/include/eigen3 -o bin/eigen_array

build:
	@mkdir -p bin 
