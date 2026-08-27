DPDK_CFLAGS = $(shell pkg-config --cflags libdpdk)
DPDK_INCLUDE_FLAGS = $(shell pkg-config --cflags-only-I libdpdk)
DPDK_LIBS = $(shell pkg-config --libs libdpdk)

CXX = g++
NVCC = nvcc

CPU_ARCH ?= native
CUDA_ARCH ?= sm_61
CUDA_HOME ?= /usr/local/cuda

# Compiler settings to guarantee optimal user-space execution performance
CXXFLAGS = -O3 -march=$(CPU_ARCH) -ffast-math -funroll-loops $(DPDK_CFLAGS) -I./h -std=c++14 -Wall
NVCCFLAGS = -O3 -arch=$(CUDA_ARCH) $(DPDK_INCLUDE_FLAGS) -I./h -std=c++14 -Xcompiler -fPIC
LDFLAGS = $(DPDK_LIBS) -L$(CUDA_HOME)/lib64 -lcudart -lpthread

TARGET = /tmp/decoder_dpdk

# Declares orchestration targets that do not map to generated output files
.PHONY: all clean

all: $(TARGET)

$(TARGET): cu/decoder.cu cpp/decoder.cpp h/decoder.h
	@$(NVCC) $(NVCCFLAGS) -c cu/decoder.cu -o /tmp/decoder_cu.o
	@$(CXX) $(CXXFLAGS) -c cpp/decoder.cpp -o /tmp/decoder_cpp.o
	@$(CXX) -o $(TARGET) /tmp/decoder_cpp.o /tmp/decoder_cu.o $(LDFLAGS)

clean:
	@rm -f /tmp/decoder_*.o $(TARGET)