DPDK_CFLAGS = $(shell pkg-config --cflags libdpdk)
DPDK_LIBS = $(shell pkg-config --libs libdpdk)

CXX = g++
NVCC = nvcc

CPU_ARCH ?= native
CUDA_ARCH ?= sm_61
CUDA_HOME ?= /usr/local/cuda

# Compiler settings to guarantee optimal user-space execution performance 
CXXFLAGS = -O3 -march=$(CPU_ARCH) -ffast-math -funroll-loops $(DPDK_CFLAGS) -I./h -std=c++14 -Wall
NVCCFLAGS = -O3 -arch=$(CUDA_ARCH) -I./h -std=c++14 -Xcompiler -fPIC
LDFLAGS = $(DPDK_LIBS) -L$(CUDA_HOME)/lib64 -lcudart -lpthread

TARGET = /tmp/encoder_dpdk

# Declares orchestration targets that do not map to generated output files
.PHONY: all clean

all: $(TARGET)

$(TARGET): cu/encoder.cu cpp/encoder.cpp
	@$(NVCC) $(NVCCFLAGS) -c cu/encoder.cu -o /tmp/encoder_cu.o
	@$(CXX) $(CXXFLAGS) -c cpp/encoder.cpp -o /tmp/encoder_cpp.o
	@$(CXX) -o $(TARGET) /tmp/encoder_cpp.o /tmp/encoder_cu.o $(LDFLAGS)

clean:
	@rm -f /tmp/*.o $(TARGET)