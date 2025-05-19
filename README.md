# DeepSleep: RISC-V Based 5-Stage Pipeline CPU Design

> Team mumbers: Jinqi Wen, Yuchao Qin, Yukun Wang (lexicographical order)

This project implements a 5-stage pipeline CPU based on the RISC-V instruction set. The design aims to deepen understanding of microprocessor architecture and pipeline techniques while laying a foundation for future intelligent chip development.

## Design Features

- **Instruction Set**: Supports the RV32I base instruction set.
- **Pipeline Stages**: Instruction Fetch (IF), Instruction Decode (ID), Execute (EX), Memory Access (MEM), Write Back (WB).
- **Hazard Handling**: Implements data forwarding and basic branch prediction.
- **Modular Design**: Independent functional modules for ease of testing and expansion.

## Directory Structure

- `src/`: Core CPU design code.
- `cdp-test/`: Test cases and simulation scripts.
- `doc/`: Design documentation and related resources.
- `Supplementary/`: some basic operation guidance

## Requirements

- A Verilog/SystemVerilog simulation tool (e.g., Icarus Verilog, ModelSim, Verilator, etc.)
- Git installed

## Quick Start

1. Clone this repository:

    ```bash
    git clone https://github.com/yourusername/mySoC.git
    cd mySoC
    ```

2. Navigate to the `cdp-test/` directory and compile the source code using the simulation tool:

    ```bash
    cd cdp-test
    make
    ```

3. Run tests:

   1. **Single Instruction Test**:
      List available test cases:
      
      ```bash
      ls bin
      ```
      
      For example, to run the test for the `sltu` instruction:
      
      ```bash
      make run TEST=sltu
      ```

   2. **Full Instruction Test Suite**:
      Use the automated Python script to run all tests:
      
      ```bash
      python3 run_all_tests.py
      ```

4. (Optional) View waveform files:

    ```bash
    gtkwave waveform/sltu.vcd
    ```

If you encounter issues, refer to the documentation in the `doc/` directory or submit an issue.

## References

- [Official RISC-V Documentation](https://riscv.org/technical/specifications/)
- *Computer Organization and Design: The Hardware/Software Interface (5th Edition, RISC-V)*

For questions or suggestions, feel free to submit an issue or a pull request.

P.s. This project is a submission for the "Jingyeda Cup" competition.