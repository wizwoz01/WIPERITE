# ---- toolchain ----------------------------------------------------------------
CROSS     ?= arm-linux-gnueabihf-
CC        := $(CROSS)gcc
OBJCOPY   := $(CROSS)objcopy
STRIP     := $(CROSS)strip

# ---- target/output -------------------------------------------------------------
TARGET    := my_firmware
BUILD     := build
SRC_DIR   := src
INC_DIR   := include
SRCS      := $(wildcard $(SRC_DIR)/*.c)
OBJS      := $(patsubst $(SRC_DIR)/%.c,$(BUILD)/%.o,$(SRCS))
ELF       := $(BUILD)/$(TARGET).elf
ELF_STRIP := $(BUILD)/$(TARGET).strip.elf

# ---- board connection ----------------------------------------------------------
BOARD_USER ?= wiperite
BOARD_HOST ?= 192.168.191.114
BOARD_PORT ?= 22
REMOTE_DIR ?= /tmp/fw
PORT       ?= 3333

# ---- flags --------------------------------------------------------------------
CFLAGS   := -I$(INC_DIR) -g3 -Og -fno-omit-frame-pointer -Wall -Wextra
LDFLAGS  := 
LDLIBS   := 

# ---- rules --------------------------------------------------------------------
.PHONY: all clean run deploy gdbserver gdb rsync

all: $(ELF)

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/%.o: $(SRC_DIR)/%.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(ELF): $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) $(LDLIBS) -o $@
	@echo "Built $@"

# optional stripped ELF for running on target (host keeps symbols)
$(ELF_STRIP): $(ELF)
	$(OBJCOPY) --only-keep-debug $(ELF) $(BUILD)/$(TARGET).debug
	$(STRIP) --strip-debug $(ELF)
	$(OBJCOPY) --add-gnu-debuglink=$(BUILD)/$(TARGET).debug $(ELF)
	cp $(ELF) $(ELF_STRIP)

clean:
	rm -rf $(BUILD)

# ---- deploy & run --------------------------------------------------------------
deploy: $(ELF_STRIP)
	ssh -p $(BOARD_PORT) $(BOARD_USER)@$(BOARD_HOST) "mkdir -p $(REMOTE_DIR)"
	scp -P $(BOARD_PORT) $(ELF_STRIP) $(BOARD_USER)@$(BOARD_HOST):$(REMOTE_DIR)/$(TARGET).elf

gdbserver: deploy
		ssh -p $(BOARD_PORT) $(BOARD_USER)@$(BOARD_HOST) "\
			killall gdbserver 2>/dev/null || true; \
			/usr/local/bin/gdbserver --once :$(PORT) $(REMOTE_DIR)/$(TARGET).elf"

# opens gdb-multiarch locally with your (symbol-full) host ELF
gdb: $(ELF)
	"/c/msys64/mingw64/bin/gdb-multiarch.exe" "$(ELF)"

# convenience: build, deploy, start gdbserver and open GDB (attach flow)
run: all deploy
		-ssh -p $(BOARD_PORT) $(BOARD_USER)@$(BOARD_HOST) "killall gdbserver 2>/dev/null || true"
		ssh -f -p $(BOARD_PORT) $(BOARD_USER)@$(BOARD_HOST) "/usr/local/bin/gdbserver --once :$(PORT) $(REMOTE_DIR)/$(TARGET).elf" \
			&& "/c/msys64/mingw64/bin/gdb-multiarch.exe" "$(ELF)" -ex "set sysroot /" \
				 -ex "target remote $(BOARD_HOST):$(PORT)" -ex "b main" -ex "c"
