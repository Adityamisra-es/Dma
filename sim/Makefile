# ==============================================================================
# Makefile  –  DMA Controller Simulation (Icarus Verilog)
# Location: e:/dma/sim/
# Usage:
#   make all        – compile and run all testbenches
#   make tb_NAME    – compile and run one testbench (e.g., make tb_dma_registers)
#   make clean      – remove generated files
# ==============================================================================

IVERILOG = C:/iverilog/bin/iverilog.exe
VVP      = C:/iverilog/bin/vvp.exe
GTKWAVE  = C:/iverilog/gtkwave/bin/gtkwave.exe

SIMFLAGS = -g2005-sv -Wall

RTL_DIR  = ../rtl
TB_DIR   = ../tb
SIM_DIR  = .

RTL_SRCS = $(RTL_DIR)/dma_fifo.v            \
            $(RTL_DIR)/dma_address_generator.v \
            $(RTL_DIR)/dma_arbiter.v           \
            $(RTL_DIR)/dma_interrupt_controller.v \
            $(RTL_DIR)/dma_axi_lite_slave.v    \
            $(RTL_DIR)/dma_burst_engine.v      \
            $(RTL_DIR)/dma_controller.v        \
            $(RTL_DIR)/dma_top.v

TB_MODEL = $(TB_DIR)/axi_memory_model.v

# All testbench targets
TB_LIST = tb_dma_registers \
          tb_dma_single_transfer \
          tb_dma_burst_transfer \
          tb_dma_errors \
          tb_dma_multichannel

.PHONY: all clean $(TB_LIST)

all: $(TB_LIST)
	@echo ""
	@echo "===== All testbenches complete ====="

# tb_dma_registers only needs the slave module
tb_dma_registers:
	@echo ""
	@echo "----- Compiling $@ -----"
	$(IVERILOG) $(SIMFLAGS) -o $@.vvp \
	    $(RTL_DIR)/dma_axi_lite_slave.v \
	    $(TB_DIR)/$@.sv -s $@
	@echo "----- Running $@ -----"
	$(VVP) $@.vvp
	@echo ""

# Full-system testbenches need all RTL + memory model
tb_dma_single_transfer tb_dma_burst_transfer tb_dma_errors tb_dma_multichannel:
	@echo ""
	@echo "----- Compiling $@ -----"
	$(IVERILOG) $(SIMFLAGS) -o $@.vvp \
	    $(RTL_SRCS) $(TB_MODEL) \
	    $(TB_DIR)/$@.sv -s $@
	@echo "----- Running $@ -----"
	$(VVP) $@.vvp
	@echo ""

clean:
	del /Q *.vvp *.vcd 2>nul || true
