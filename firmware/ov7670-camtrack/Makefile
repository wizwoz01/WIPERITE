\
# Simple userspace Makefile for Zynq-7000 PetaLinux
PROJECT := camtrack
BIN_DIR := bin
SRC_DIR := src

CROSS_COMPILE ?=
CXX := $(CROSS_COMPILE)g++
AR  := $(CROSS_COMPILE)ar

CXXFLAGS += -O3 -std=c++17 -Wall -Wextra -Wno-unused-parameter
LDFLAGS  += 

SOURCES := $(SRC_DIR)/camtrack.cpp
OBJECTS := $(SOURCES:.cpp=.o)
TARGET  := $(BIN_DIR)/$(PROJECT)

all: $(TARGET)

$(BIN_DIR):
\tmkdir -p $(BIN_DIR)

$(TARGET): $(BIN_DIR) $(OBJECTS)
\t$(CXX) $(CXXFLAGS) -o $@ $(OBJECTS) $(LDFLAGS)

%.o: %.cpp
\t$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
\trm -rf $(BIN_DIR) $(OBJECTS)

.PHONY: all clean
\

# --- OpenCV-enabled variant ---
OPENCV_PKG ?= opencv4
OPENCV_CFLAGS := $(shell pkg-config --cflags $(OPENCV_PKG) 2>/dev/null)
OPENCV_LIBS   := $(shell pkg-config --libs   $(OPENCV_PKG) 2>/dev/null)
ifeq ($(OPENCV_CFLAGS),)
  OPENCV_PKG := opencv
  OPENCV_CFLAGS := $(shell pkg-config --cflags $(OPENCV_PKG) 2>/dev/null)
  OPENCV_LIBS   := $(shell pkg-config --libs   $(OPENCV_PKG) 2>/dev/null)
endif

SRC_CV   := $(SRC_DIR)/camtrack_cv.cpp
OBJ_CV   := $(SRC_CV:.cpp=.o)
TARGET_CV:= $(BIN_DIR)/camtrack_cv

$(TARGET_CV): $(BIN_DIR) $(OBJ_CV)
\t@if [ -z "$(OPENCV_LIBS)" ]; then echo "OpenCV not found via pkg-config (tried opencv4/opencv)"; exit 1; fi
\t$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -o $@ $(OBJ_CV) $(OPENCV_LIBS)

$(SRC_DIR)/camtrack_cv.o: $(SRC_DIR)/camtrack_cv.cpp
\t@if [ -z "$(OPENCV_CFLAGS)" ]; then echo "OpenCV not found via pkg-config (tried opencv4/opencv)"; exit 1; fi
\t$(CXX) $(CXXFLAGS) $(OPENCV_CFLAGS) -c $< -o $@

cv: $(TARGET_CV)
