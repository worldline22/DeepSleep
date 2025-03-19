# Makefile for RISC-V CPU simulation using iverilog
# Variables
IVERILOG = iverilog
VVP = vvp
WAVE_VIEWER = gtkwave

# Source files
SRC_FILES = IF_ID.sv alu.sv mem.sv PC.sv Reg.sv top.sv
TB_FILE = top_tb.sv

# Output files
BUILD_DIR = build
COMPILED = $(BUILD_DIR)/riscv_cpu
VCD_FILE = $(BUILD_DIR)/riscv_cpu_waveform.vcd

# Targets and rules
.PHONY: all clean run sim wave

all: $(COMPILED)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile the design
$(COMPILED): $(SRC_FILES) $(TB_FILE) | $(BUILD_DIR)
	$(IVERILOG) -o $@ -g2012 $(SRC_FILES) $(TB_FILE)

# Run the simulation
run: $(COMPILED)
	$(VVP) $(COMPILED)

# Run the simulation and generate waveform file
sim: $(COMPILED)
	$(VVP) $(COMPILED) -lxt2

# View waveforms
wave: sim
	$(WAVE_VIEWER) $(VCD_FILE) &

# Clean up generated files
clean:
	rm -rf $(BUILD_DIR)

# Simple test to verify file existence
test_files:
	@echo "Checking for source files:"
	@for file in $(SRC_FILES) $(TB_FILE); do \
		if [ -f $$file ]; then \
			echo "  ✓ $$file found"; \
		else \
			echo "  ✗ $$file NOT found"; \
		fi; \
	done

# Help target
help:
	@echo "RISC-V CPU Simulation Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  all       - Compile the design (default)"
	@echo "  run       - Run the simulation"
	@echo "  sim       - Run the simulation and generate waveform file"
	@echo "  wave      - Run the simulation and open waveform viewer"
	@echo "  clean     - Remove generated files"
	@echo "  test_files - Check for existence of source files"
	@echo "  help      - Display this help message"