# Makefile for WIPERITE base-station client
# Supports native build on target or cross-compile using PetaLinux SDK.

# If CROSS_COMPILE is set (e.g., arm-linux-gnueabihf-), we'll use it
CC ?= $(CROSS_COMPILE)gcc
CFLAGS ?= -O2 -Wall -Wextra -pedantic
LDFLAGS ?=

TARGET := wiperite_client
SRC := wiperite_client.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(TARGET) *.o

.PHONY: all clean
