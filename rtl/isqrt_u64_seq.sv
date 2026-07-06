// ============================================================
// isqrt_u64_seq.sv  -  Sequential 64-bit Integer Square Root
//
// Iterative non-restoring algorithm, 32 clock cycles.
// Used by adpv2_encoder_block for sigma_y computation.
//
// Instantiated as: u_sqrt
// ============================================================
module isqrt_u64_seq (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        start,
  input  logic [63:0] x,
  output logic        busy,
  output logic        done,
  output logic [31:0] root
);

  logic [63:0] x_reg;
  logic [65:0] rem_reg;
  logic [31:0] r_reg;
  logic [5:0]  i_reg;

  logic [1:0]  bits2_w;
  logic [65:0] trial_ext_w;
  logic [65:0] rem_shift_w;
  logic [65:0] rem_next_w;
  logic [31:0] r_next_w;

  always_comb begin
    bits2_w     = (x_reg >> (62 - 2*i_reg)) & 2'b11;
    trial_ext_w = {32'd0, r_reg, 2'b01};
    rem_shift_w = (rem_reg << 2) | {64'd0, bits2_w};
    if (rem_shift_w >= trial_ext_w) begin
      rem_next_w = rem_shift_w - trial_ext_w;
      r_next_w   = (r_reg << 1) | 1'b1;
    end else begin
      rem_next_w = rem_shift_w;
      r_next_w   = (r_reg << 1);
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      x_reg <= '0; rem_reg <= '0; r_reg <= '0; i_reg <= '0;
      busy  <= 1'b0; done <= 1'b0; root <= '0;
    end else begin
      done <= 1'b0;
      if (start && !busy) begin
        x_reg <= x; rem_reg <= '0; r_reg <= '0; i_reg <= 6'd0; busy <= 1'b1;
      end else if (busy) begin
        rem_reg <= rem_next_w;
        r_reg   <= r_next_w;
        if (i_reg == 6'd31) begin
          root <= r_next_w; done <= 1'b1; busy <= 1'b0;
        end else begin
          i_reg <= i_reg + 1;
        end
      end
    end
  end

endmodule


// ============================================================================
// sdiv_round32_seq (unchanged)
// ============================================================================
(* use_dsp = "no" *)
(* retime_forward = "true" *)