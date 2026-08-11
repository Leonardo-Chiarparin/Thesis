DPDK_CFLAGS = $(shell pkg-config --cflags libdpdk)
DPDK_LIBS = $(shell pkg-config --libs libdpdk)

CXX = g++
NVCC = nvcc

# Variables to guarantee high user-space performance 
CXXFLAGS = -O3 -march=native -ffast-math -funroll-loops $(DPDK_CFLAGS) -I./h -std=c++14 -Wall
NVCCFLAGS = -O3 -arch=sm_61 -I./h -std=c++14 -Xcompiler -fPIC
LDFLAGS = $(DPDK_LIBS) -lcudart -lpthread

LDFLAGS = $(DPDK_LIBS) -L/usr/local/cuda/lib64 -lcudart -lpthread

TARGET = /tmp/encoder_dpdk

# Forces make to run these elements ignoring cached files
.PHONY: all clean

all: $(TARGET)

$(TARGET): cu/encoder.cu cpp/encoder.cpp
	@$(NVCC) $(NVCCFLAGS) -c cu/encoder.cu -o /tmp/encoder_cu.o
	@$(CXX) $(CXXFLAGS) -c cpp/encoder.cpp -o /tmp/encoder_cpp.o
	@$(CXX) -o $(TARGET) /tmp/encoder_cpp.o /tmp/encoder_cu.o $(LDFLAGS)

clean:
	@rm -f /tmp/*.o $(TARGET)