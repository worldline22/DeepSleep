I'll create a Makefile for compiling and testing your RISC-V CPU with iverilog. This will make it easier to build and run simulations.

I've created a comprehensive Makefile and a testbench to help you test your RISC-V CPU using iverilog. Here's what each file does:

### 1. Makefile

The Makefile provides several useful commands:

- `make` or `make all`: Compiles your CPU design
- `make run`: Runs the simulation and shows output in the terminal
- `make sim`: Runs the simulation and generates a waveform file
- `make wave`: Runs the simulation and opens the waveform file in GTKWave
- `make clean`: Removes all generated files
- `make test_files`: Checks if all required source files exist
- `make help`: Shows help information about available commands

### 2. Testbench (Top_tb.sv)

I've created a comprehensive testbench that:

- Instantiates your CPU top module
- Generates clock and reset signals
- Displays key CPU signals in the terminal:
  - Current time
  - Program counter (PC)
  - Current instruction
  - ALU result
- Dumps waveform data to a VCD file for viewing with GTKWave
- Shows register file contents after execution
- Shows data memory contents after execution

### How to Use:

1. Save the Makefile in the same directory as your SystemVerilog files
2. Save the Top_tb.sv file in the same directory
3. Open a terminal in that directory
4. Run one of the following commands:

```bash
# Compile the design
make

# Run the simulation and see output in terminal
make run

# Run simulation and generate waveform file
make sim

# View waveforms in GTKWave
make wave
```

### Notes:

1. The Makefile assumes your top-level module is named `top` in the file `top.sv`
2. Make sure your file names match exactly as listed in the Makefile (ALU.sv, DateMem.sv, PC.sv, Regfile.sv, Top.sv)
3. The testbench monitors key signals and displays them in the terminal
4. If you want to view waveforms, you'll need GTKWave installed

The testbench will run for 1000ns, monitoring your CPU's execution. It will display the PC, instruction, and ALU result at each clock cycle, then show the contents of the register file and data memory at specific points in time.

If you encounter any issues, you can use `make test_files` to check if all required files are present and correctly named.