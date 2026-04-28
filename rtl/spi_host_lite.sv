// Copyright 2026 Vyges Inc.
// SPDX-License-Identifier: Apache-2.0
//
// spi_host_lite — Lightweight SPI master controller with TL-UL slave interface
//
// Features:
//   - SPI modes 0–3 (CPOL/CPHA configurable)
//   - Configurable clock divider (SCLK = clk_i / (2 * (div+1)))
//   - 8/16/32-bit transfer length
//   - TX and RX FIFOs (depth configurable, default 8)
//   - Single chip-select output (active low)
//   - Interrupts: tx_empty, rx_full, idle
//
// Register map (32-bit aligned):
//   0x00  CTRL     — [0] enable, [1] cpol, [2] cpha, [4:3] xfer_len (0=8,1=16,2=32)
//   0x04  STATUS   — [0] busy, [1] tx_full, [2] tx_empty, [3] rx_full, [4] rx_empty
//   0x08  DIV      — [15:0] clock divider value
//   0x0C  TXDATA   — [31:0] write to push TX FIFO
//   0x10  RXDATA   — [31:0] read to pop RX FIFO
//   0x14  CSCTRL   — [0] cs_assert (1 = drive CS low)
//   0x18  INTR_EN  — [0] tx_empty_en, [1] rx_full_en, [2] idle_en
//   0x1C  INTR_ST  — [0] tx_empty, [1] rx_full, [2] idle (write-1-to-clear)

`ifndef SPI_HOST_LITE_SV
`define SPI_HOST_LITE_SV

module spi_host_lite
  import tlul_pkg::*;
#(
  parameter int unsigned FIFO_DEPTH = 8
) (
  input  logic clk_i,
  input  logic rst_ni,

  // TL-UL slave interface
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,

  // SPI master signals
  output logic spi_sclk_o,
  output logic spi_cs_no,
  output logic spi_mosi_o,
  input  logic spi_miso_i,

  // Interrupt outputs
  output logic intr_tx_empty_o,
  output logic intr_rx_full_o,
  output logic intr_idle_o
);

  // -------------------------------------------------------------------------
  // TL-UL response logic
  // -------------------------------------------------------------------------

  logic        req_valid;
  logic        req_write;
  logic [31:0] req_addr;
  logic [31:0] req_wdata;
  logic [3:0]  req_mask;
  logic [7:0]  req_source;
  logic [1:0]  req_size;

  logic        rsp_valid;
  logic [31:0] rsp_rdata;
  logic        rsp_error;

  // Latched source/size/opcode for the d-channel response (driven below).
  // Declared up front so the tl_o_pre.d_* assigns don't trip Synth 8-6901
  // (use-before-declaration) — Vivado's strict-lint pass flags forward
  // refs even when the surrounding code is functionally fine.
  logic [7:0] rsp_source_q;
  logic [1:0] rsp_size_q;
  logic       rsp_write_q;

  // A channel unpack
  assign req_valid  = tl_i.a_valid;
  assign req_write  = (tl_i.a_opcode == tlul_pkg::PutFullData) || (tl_i.a_opcode == tlul_pkg::PutPartialData);
  assign req_addr   = tl_i.a_address;
  assign req_wdata  = tl_i.a_data;
  assign req_mask   = tl_i.a_mask;
  assign req_source = tl_i.a_source;
  assign req_size   = tl_i.a_size;

  // Single-cycle response (always ready)
  // Build pre-integrity response; tlul_rsp_intg_gen below signs rsp_intg +
  // data_intg so CPU-side tlul_rsp_intg_chk (always-on in opentitan-rv-core-
  // ibex's tlul_adapter_host) accepts the d-channel. Without this, any CPU
  // transaction to SPI traps Ibex on the first response. See the Bus
  // security domain contract work item in deckrun-server/docs/todo.md.
  tl_d2h_t tl_o_pre;
  assign tl_o_pre.a_ready  = 1'b1;
  assign tl_o_pre.d_valid  = rsp_valid;
  // Use LATCHED _q versions — req_write / req_size / req_source track the
  // current cycle's a_channel input, but rsp_valid is pipelined 1 cycle so
  // by the time we assert d_valid, a_source has moved on. Response routed
  // via d_source gets delivered to the wrong host (SBA requests from DM
  // master get responses delivered to CPU instead of DM). Original
  // assignments (req_source, req_write, req_size directly) were a latent
  // bug that only surfaced under a multi-host signed TL-UL domain.
  assign tl_o_pre.d_opcode = tl_d_op_e'(rsp_write_q ? AccessAck : AccessAckData);
  assign tl_o_pre.d_param  = '0;
  assign tl_o_pre.d_size   = rsp_size_q;
  assign tl_o_pre.d_source = rsp_source_q;
  assign tl_o_pre.d_sink   = '0;
  assign tl_o_pre.d_data   = rsp_rdata;
  assign tl_o_pre.d_error  = rsp_error;
  assign tl_o_pre.d_user   = '0;

  tlul_rsp_intg_gen #(
    .EnableRspIntgGen  (1),
    .EnableDataIntgGen (1)
  ) u_rsp_intg_gen (
    .tl_i (tl_o_pre),
    .tl_o (tl_o)
  );

  // Latch source/size/write for the d-channel response (declarations are
  // above near the rest of the rsp_* signals).
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rsp_valid  <= 1'b0;
      rsp_source_q <= '0;
      rsp_size_q   <= '0;
      rsp_write_q  <= 1'b0;
    end else begin
      rsp_valid    <= req_valid;
      rsp_source_q <= req_source;
      rsp_size_q   <= req_size;
      rsp_write_q  <= req_write;
    end
  end

  // -------------------------------------------------------------------------
  // Registers
  // -------------------------------------------------------------------------

  logic        reg_enable;
  logic        reg_cpol;
  logic        reg_cpha;
  logic [1:0]  reg_xfer_len;   // 0=8bit, 1=16bit, 2=32bit
  // CTRL[5] — internal MOSI->MISO loopback for self-test. When set, the
  // shift_rx samples the MOSI output we are driving rather than the
  // external spi_miso_i pin. Lets any SoC run a SPI host self-test via
  // SBA without needing a physical peripheral or a jumper on the Pmod.
  // Default 0 (external mode). See fpga_day0_spi_loopback_*.py.
  logic        reg_loopback;
  logic [15:0] reg_div;
  logic        reg_cs_assert;
  logic [2:0]  reg_intr_en;
  logic [2:0]  reg_intr_st;

  // FIFO signals
  logic        tx_push, tx_pop, tx_full, tx_empty;
  logic [31:0] tx_wdata, tx_rdata;
  logic        rx_push, rx_pop, rx_full, rx_empty;
  logic [31:0] rx_wdata, rx_rdata;

  // SPI engine state
  logic        spi_busy;
  logic        spi_idle_edge;   // rising edge of idle

  // -------------------------------------------------------------------------
  // Register write
  // -------------------------------------------------------------------------

  logic [7:0] reg_offset;
  assign reg_offset = req_addr[7:0];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      reg_enable    <= 1'b0;
      reg_cpol      <= 1'b0;
      reg_cpha      <= 1'b0;
      reg_xfer_len  <= 2'b0;
      reg_loopback  <= 1'b0;
      reg_div       <= 16'd7;   // default: clk/16
      reg_cs_assert <= 1'b0;
      reg_intr_en   <= 3'b0;
      reg_intr_st   <= 3'b0;
    end else begin
      // Interrupt status set
      if (tx_empty)     reg_intr_st[0] <= 1'b1;
      if (rx_full)      reg_intr_st[1] <= 1'b1;
      if (spi_idle_edge) reg_intr_st[2] <= 1'b1;

      if (req_valid && req_write) begin
        case (reg_offset)
          8'h00: begin  // CTRL
            reg_enable   <= req_wdata[0];
            reg_cpol     <= req_wdata[1];
            reg_cpha     <= req_wdata[2];
            reg_xfer_len <= req_wdata[4:3];
            reg_loopback <= req_wdata[5];   // CTRL[5] — internal loopback
          end
          8'h08: reg_div       <= req_wdata[15:0];  // DIV
          8'h14: reg_cs_assert <= req_wdata[0];      // CSCTRL
          8'h18: reg_intr_en   <= req_wdata[2:0];    // INTR_EN
          8'h1C: reg_intr_st   <= reg_intr_st & ~req_wdata[2:0];  // INTR_ST (W1C)
          default: ;
        endcase
      end
    end
  end

  // TX FIFO push on write to TXDATA
  assign tx_push  = req_valid && req_write && (reg_offset == 8'h0C) && !tx_full;
  assign tx_wdata = req_wdata;

  // RX FIFO pop on read from RXDATA
  assign rx_pop = req_valid && !req_write && (reg_offset == 8'h10) && !rx_empty;

  // -------------------------------------------------------------------------
  // Register read
  // -------------------------------------------------------------------------

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rsp_rdata <= '0;
      rsp_error <= 1'b0;
    end else if (req_valid && !req_write) begin
      rsp_error <= 1'b0;
      case (reg_offset)
        8'h00: rsp_rdata <= {26'b0, reg_loopback, reg_xfer_len, reg_cpha, reg_cpol, reg_enable};
        8'h04: rsp_rdata <= {27'b0, rx_empty, rx_full, tx_empty, tx_full, spi_busy};
        8'h08: rsp_rdata <= {16'b0, reg_div};
        8'h10: rsp_rdata <= rx_rdata;
        8'h14: rsp_rdata <= {31'b0, reg_cs_assert};
        8'h18: rsp_rdata <= {29'b0, reg_intr_en};
        8'h1C: rsp_rdata <= {29'b0, reg_intr_st};
        default: begin
          rsp_rdata <= '0;
          rsp_error <= 1'b1;
        end
      endcase
    end else begin
      rsp_rdata <= '0;
      rsp_error <= 1'b0;
    end
  end

  // -------------------------------------------------------------------------
  // Interrupt outputs
  // -------------------------------------------------------------------------

  assign intr_tx_empty_o = reg_intr_en[0] & reg_intr_st[0];
  assign intr_rx_full_o  = reg_intr_en[1] & reg_intr_st[1];
  assign intr_idle_o     = reg_intr_en[2] & reg_intr_st[2];

  // -------------------------------------------------------------------------
  // TX FIFO
  // -------------------------------------------------------------------------

  logic [$clog2(FIFO_DEPTH):0] tx_count;
  logic [31:0] tx_mem [FIFO_DEPTH];
  logic [$clog2(FIFO_DEPTH)-1:0] tx_wptr, tx_rptr;

  assign tx_full  = (tx_count == FIFO_DEPTH[$clog2(FIFO_DEPTH):0]);
  assign tx_empty = (tx_count == '0);
  assign tx_rdata = tx_mem[tx_rptr];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tx_wptr  <= '0;
      tx_rptr  <= '0;
      tx_count <= '0;
      // Explicit reset of FIFO storage so Vivado's priority analysis
      // sees reset > conditional-write unambiguously. Without this,
      // Vivado treats the absence of an assignment in the reset branch
      // as an implicit "hold current value" with the same priority as
      // the conditional `tx_mem[tx_wptr] <= tx_wdata` in the else
      // branch, and emits Synth 8-7137 (set/reset same priority) per
      // FIFO entry. FPGA cost is zero (Xilinx FFs init via GSR
      // regardless); ASIC cost is FIFO_DEPTH × 32 reset wires.
      for (int i = 0; i < FIFO_DEPTH; i++) tx_mem[i] <= '0;
    end else begin
      if (tx_push && !tx_pop) begin
        tx_mem[tx_wptr] <= tx_wdata;
        tx_wptr  <= tx_wptr + 1'b1;
        tx_count <= tx_count + 1'b1;
      end else if (!tx_push && tx_pop) begin
        tx_rptr  <= tx_rptr + 1'b1;
        tx_count <= tx_count - 1'b1;
      end else if (tx_push && tx_pop) begin
        tx_mem[tx_wptr] <= tx_wdata;
        tx_wptr <= tx_wptr + 1'b1;
        tx_rptr <= tx_rptr + 1'b1;
      end
    end
  end

  // -------------------------------------------------------------------------
  // RX FIFO
  // -------------------------------------------------------------------------

  logic [$clog2(FIFO_DEPTH):0] rx_count;
  logic [31:0] rx_mem [FIFO_DEPTH];
  logic [$clog2(FIFO_DEPTH)-1:0] rx_wptr, rx_rptr;

  assign rx_full  = (rx_count == FIFO_DEPTH[$clog2(FIFO_DEPTH):0]);
  assign rx_empty = (rx_count == '0);
  assign rx_rdata = rx_mem[rx_rptr];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rx_wptr  <= '0;
      rx_rptr  <= '0;
      rx_count <= '0;
      // Explicit reset of FIFO storage — see tx_mem block above for the
      // Synth 8-7137 priority-disambiguation rationale.
      for (int i = 0; i < FIFO_DEPTH; i++) rx_mem[i] <= '0;
    end else begin
      if (rx_push && !rx_pop) begin
        rx_mem[rx_wptr] <= rx_wdata;
        rx_wptr  <= rx_wptr + 1'b1;
        rx_count <= rx_count + 1'b1;
      end else if (!rx_push && rx_pop) begin
        rx_rptr  <= rx_rptr + 1'b1;
        rx_count <= rx_count - 1'b1;
      end else if (rx_push && rx_pop) begin
        rx_mem[rx_wptr] <= rx_wdata;
        rx_wptr <= rx_wptr + 1'b1;
        rx_rptr <= rx_rptr + 1'b1;
      end
    end
  end

  // -------------------------------------------------------------------------
  // SPI shift engine
  // -------------------------------------------------------------------------

  typedef enum logic [1:0] {
    ST_IDLE,
    ST_LOAD,
    ST_SHIFT,
    ST_DONE
  } spi_state_e;

  spi_state_e state_q, state_d;

  logic [15:0] clk_cnt;
  logic        clk_edge;     // toggles at divided rate
  logic        sclk_q;
  logic [31:0] shift_tx, shift_rx;
  logic [5:0]  bit_cnt;
  logic [5:0]  bit_max;
  logic        spi_busy_prev;
  // CPHA=1 modes (1 and 3) need MOSI held for one half-clock after LOAD
  // before the first master-shift so the slave sees bit 7 on its first
  // sample edge. first_leading_skip: set in ST_LOAD when reg_cpha=1; the
  // first leading edge in ST_SHIFT clears it without shifting. Subsequent
  // leading edges shift normally. In CPHA=0 this stays 0 and has no effect.
  logic        first_leading_skip;

  // Bit count based on transfer length
  always_comb begin
    case (reg_xfer_len)
      2'd0: bit_max = 6'd7;    // 8-bit
      2'd1: bit_max = 6'd15;   // 16-bit
      2'd2: bit_max = 6'd31;   // 32-bit
      default: bit_max = 6'd7;
    endcase
  end

  // Clock divider
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      clk_cnt  <= '0;
      clk_edge <= 1'b0;
    end else if (state_q == ST_SHIFT) begin
      if (clk_cnt == reg_div) begin
        clk_cnt  <= '0;
        clk_edge <= 1'b1;
      end else begin
        clk_cnt  <= clk_cnt + 1'b1;
        clk_edge <= 1'b0;
      end
    end else begin
      clk_cnt  <= '0;
      clk_edge <= 1'b0;
    end
  end

  // SPI FSM
  assign tx_pop  = (state_q == ST_IDLE) && !tx_empty && reg_enable;
  assign rx_push = (state_q == ST_DONE) && !rx_full;
  assign rx_wdata = shift_rx;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q   <= ST_IDLE;
      sclk_q    <= 1'b0;
      shift_tx  <= '0;
      shift_rx  <= '0;
      bit_cnt   <= '0;
      spi_busy_prev <= 1'b0;
      first_leading_skip <= 1'b0;
    end else begin
      spi_busy_prev <= spi_busy;

      case (state_q)
        ST_IDLE: begin
          sclk_q <= reg_cpol;
          if (!tx_empty && reg_enable) begin
            // Align tx_rdata to shift_tx MSB based on reg_xfer_len so
            // MOSI (= shift_tx[31]) emits real data on the first edge
            // and each shift-left drops the next bit out. Without this
            // alignment, an 8-bit transfer of 0x5B (loaded as
            // 0x0000005B) would shift 0s out of bit 31 for the entire
            // 8-clock window -- MOSI always 0, no matter what the CPU
            // wrote to TXDATA. Masked by the external MISO behavior
            // (sensor saw only 0x00 command bytes and never responded;
            // we read floating MISO = 0xFF and blamed the sensor).
            // Discovered during FPGA loopback diagnostic 2026-04-18.
            unique case (reg_xfer_len)
              2'd0: shift_tx <= {tx_rdata[ 7:0], 24'b0};   // 8-bit
              2'd1: shift_tx <= {tx_rdata[15:0], 16'b0};   // 16-bit
              2'd2: shift_tx <= tx_rdata;                  // 32-bit
              default: shift_tx <= {tx_rdata[7:0], 24'b0};
            endcase
            shift_rx <= '0;
            bit_cnt  <= '0;
            state_q  <= ST_LOAD;
          end
        end

        ST_LOAD: begin
          // One cycle to latch data, then shift. In CPHA=1 modes the
          // first leading edge must not shift -- slave samples bit 7 on
          // the first trailing edge and MOSI must still hold bit 7.
          first_leading_skip <= reg_cpha;
          state_q <= ST_SHIFT;
        end

        ST_SHIFT: begin
          if (clk_edge) begin
            // MISO source mux: external pin in normal mode, MOSI echo in
            // loopback mode. MOSI is shift_tx[31] -- the bit we're about
            // to (or just did) drive on the external pin.
            automatic logic miso_sample = reg_loopback ? shift_tx[31] : spi_miso_i;
            if (!sclk_q ^ reg_cpol) begin
              // Leading edge: sample MISO (CPHA=0) or shift MOSI (CPHA=1)
              if (!reg_cpha) begin
                shift_rx <= {shift_rx[30:0], miso_sample};
              end else begin
                // CPHA=1: skip the very first leading-edge shift so MOSI
                // still holds bit 7 when the slave samples on the upcoming
                // trailing edge. 2nd+ leading edges shift normally.
                if (first_leading_skip) begin
                  first_leading_skip <= 1'b0;
                end else begin
                  shift_tx <= {shift_tx[30:0], 1'b0};
                end
              end
              sclk_q <= ~sclk_q;
            end else begin
              // Trailing edge: shift MOSI (CPHA=0) or sample MISO (CPHA=1)
              if (!reg_cpha) begin
                shift_tx <= {shift_tx[30:0], 1'b0};
              end else begin
                shift_rx <= {shift_rx[30:0], miso_sample};
              end
              sclk_q <= ~sclk_q;

              if (bit_cnt == bit_max) begin
                state_q <= ST_DONE;
              end else begin
                bit_cnt <= bit_cnt + 1'b1;
              end
            end
          end
        end

        ST_DONE: begin
          sclk_q <= reg_cpol;
          state_q <= ST_IDLE;
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

  assign spi_busy = (state_q != ST_IDLE);
  assign spi_idle_edge = spi_busy_prev && !spi_busy;

  // SPI output signals
  assign spi_sclk_o = sclk_q;
  assign spi_cs_no  = ~reg_cs_assert;
  assign spi_mosi_o = shift_tx[31];  // MSB first

endmodule

`endif // SPI_HOST_LITE_SV
