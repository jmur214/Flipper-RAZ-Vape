# Makefile — build Vaporware firmware examples for N32G031K8Q7-1
#
# Requires: arm-none-eabi-gcc in PATH (sudo apt install gcc-arm-none-eabi on Ubuntu)
# Usage:
#   make flappy          → firmware/flappy.bin
#   make slots           → firmware/slots.bin
#   make all             → all of the above
#   make clean

CROSS    := arm-none-eabi-
CC       := $(CROSS)gcc
OBJCOPY  := $(CROSS)objcopy
SIZE     := $(CROSS)size

CFLAGS := \
    -mcpu=cortex-m0 \
    -mthumb \
    -Os \
    -Wall \
    -ffunction-sections \
    -fdata-sections \
    -fno-common \
    -Wno-misleading-indentation \
    -Wno-unused-function \
    -Ivaporware/src/include

LDFLAGS_BASE := \
    -mcpu=cortex-m0 \
    -mthumb \
    -Wl,--gc-sections \
    -T vaporware/src/n32g031.ld \
    -nostartfiles

# Vaporware library sources (shared by all examples)
VAPORWARE_SRC := \
    vaporware/src/src/startup.s \
    vaporware/src/src/system.c \
    vaporware/src/src/display.c \
    vaporware/src/src/button.c \
    vaporware/src/src/battery.c \
    vaporware/src/src/vape.c \
    vaporware/src/src/nv.c \
    vaporware/src/src/app.c

$(shell mkdir -p firmware)

# ── flappy ────────────────────────────────────────────────────────────────────
firmware/flappy.elf: $(VAPORWARE_SRC) vaporware/examples/flappy/src/flappy.c
	$(CC) $(CFLAGS) $(LDFLAGS_BASE) -Wl,-Map,$(@:.elf=.map) $^ -o $@
	$(SIZE) $@

firmware/flappy.bin: firmware/flappy.elf
	$(OBJCOPY) -O binary $< $@
	@echo "  → $@ ($$(wc -c < $@) bytes)"

.PHONY: flappy
flappy: firmware/flappy.bin

# ── slots ─────────────────────────────────────────────────────────────────────
firmware/slots.elf: $(VAPORWARE_SRC) vaporware/examples/slots/src/slots.c vaporware/examples/slots/src/main.c
	$(CC) $(CFLAGS) -Ivaporware/examples/slots/include $(LDFLAGS_BASE) -Wl,-Map,$(@:.elf=.map) $^ -o $@
	$(SIZE) $@

firmware/slots.bin: firmware/slots.elf
	$(OBJCOPY) -O binary $< $@
	@echo "  → $@ ($$(wc -c < $@) bytes)"

.PHONY: slots
slots: firmware/slots.bin

# ── template ──────────────────────────────────────────────────────────────────
firmware/template.elf: $(VAPORWARE_SRC) vaporware/examples/template/src/main.c
	$(CC) $(CFLAGS) $(LDFLAGS_BASE) -Wl,-Map,$(@:.elf=.map) $^ -o $@
	$(SIZE) $@

firmware/template.bin: firmware/template.elf
	$(OBJCOPY) -O binary $< $@
	@echo "  → $@ ($$(wc -c < $@) bytes)"

.PHONY: template
template: firmware/template.bin

# ── dino ──────────────────────────────────────────────────────────────────────
firmware/dino.elf: $(VAPORWARE_SRC) vaporware/examples/dino/src/dino.c
	$(CC) $(CFLAGS) $(LDFLAGS_BASE) -Wl,-Map,$(@:.elf=.map) $^ -o $@
	$(SIZE) $@

firmware/dino.bin: firmware/dino.elf
	$(OBJCOPY) -O binary $< $@
	@echo "  → $@ ($$(wc -c < $@) bytes)"

.PHONY: dino
dino: firmware/dino.bin

# ── micro ─────────────────────────────────────────────────────────────────────
firmware/micro.elf: $(VAPORWARE_SRC) vaporware/examples/micro/src/micro.c
	$(CC) $(CFLAGS) $(LDFLAGS_BASE) -Wl,-Map,$(@:.elf=.map) $^ -o $@
	$(SIZE) $@

firmware/micro.bin: firmware/micro.elf
	$(OBJCOPY) -O binary $< $@
	@echo "  → $@ ($$(wc -c < $@) bytes)"

.PHONY: micro
micro: firmware/micro.bin

# ── bomb ──────────────────────────────────────────────────────────────────────
firmware/bomb.elf: $(VAPORWARE_SRC) vaporware/examples/bomb/src/bomb.c
	$(CC) $(CFLAGS) $(LDFLAGS_BASE) -Wl,-Map,$(@:.elf=.map) $^ -o $@
	$(SIZE) $@

firmware/bomb.bin: firmware/bomb.elf
	$(OBJCOPY) -O binary $< $@
	@echo "  → $@ ($$(wc -c < $@) bytes)"

.PHONY: bomb
bomb: firmware/bomb.bin

# ── tetris ────────────────────────────────────────────────────────────────────
firmware/tetris.elf: $(VAPORWARE_SRC) vaporware/examples/tetris/src/tetris.c
	$(CC) $(CFLAGS) $(LDFLAGS_BASE) -Wl,-Map,$(@:.elf=.map) $^ -o $@
	$(SIZE) $@

firmware/tetris.bin: firmware/tetris.elf
	$(OBJCOPY) -O binary $< $@
	@echo "  → $@ ($$(wc -c < $@) bytes)"

.PHONY: tetris
tetris: firmware/tetris.bin

# ── quickdraw ─────────────────────────────────────────────────────────────────
firmware/quickdraw.elf: $(VAPORWARE_SRC) vaporware/examples/quickdraw/src/quickdraw.c
	$(CC) $(CFLAGS) $(LDFLAGS_BASE) -Wl,-Map,$(@:.elf=.map) $^ -o $@
	$(SIZE) $@

firmware/quickdraw.bin: firmware/quickdraw.elf
	$(OBJCOPY) -O binary $< $@
	@echo "  → $@ ($$(wc -c < $@) bytes)"

.PHONY: quickdraw
quickdraw: firmware/quickdraw.bin

# ── dare ──────────────────────────────────────────────────────────────────────
# deck.h is a prerequisite so editing the cards actually triggers a rebuild;
# $^ would then hand the header to gcc as a source, so filter to sources only.
firmware/dare.elf: $(VAPORWARE_SRC) vaporware/examples/dare/src/dare.c vaporware/examples/dare/src/deck.h
	$(CC) $(CFLAGS) -Ivaporware/examples/dare/src $(LDFLAGS_BASE) -Wl,-Map,$(@:.elf=.map) $(filter %.c %.s,$^) -o $@
	$(SIZE) $@

firmware/dare.bin: firmware/dare.elf
	$(OBJCOPY) -O binary $< $@
	@echo "  → $@ ($$(wc -c < $@) bytes)"

.PHONY: dare
dare: firmware/dare.bin

# ── kings ─────────────────────────────────────────────────────────────────────
firmware/kings.elf: $(VAPORWARE_SRC) vaporware/examples/kings/src/kings.c
	$(CC) $(CFLAGS) $(LDFLAGS_BASE) -Wl,-Map,$(@:.elf=.map) $(filter %.c %.s,$^) -o $@
	$(SIZE) $@

firmware/kings.bin: firmware/kings.elf
	$(OBJCOPY) -O binary $< $@
	@echo "  → $@ ($$(wc -c < $@) bytes)"

.PHONY: kings
kings: firmware/kings.bin

# ── hour ──────────────────────────────────────────────────────────────────────
firmware/hour.elf: $(VAPORWARE_SRC) vaporware/examples/hour/src/hour.c
	$(CC) $(CFLAGS) $(LDFLAGS_BASE) -Wl,-Map,$(@:.elf=.map) $(filter %.c %.s,$^) -o $@
	$(SIZE) $@

firmware/hour.bin: firmware/hour.elf
	$(OBJCOPY) -O binary $< $@
	@echo "  → $@ ($$(wc -c < $@) bytes)"

.PHONY: hour
hour: firmware/hour.bin

# ── all / clean ───────────────────────────────────────────────────────────────
.PHONY: all
all: flappy slots template dino micro bomb tetris quickdraw dare kings hour

.PHONY: clean
clean:
	rm -rf firmware/*.elf firmware/*.bin firmware/*.map
