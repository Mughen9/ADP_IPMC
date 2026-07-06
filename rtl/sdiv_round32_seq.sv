// ============================================================
// sdiv_round32_seq.sv  -  Sequential Signed 64-bit Divider
//
// 64 clock cycle iterative long division with rounding.
// Critical path module - limits max frequency to 98 MHz on Artix-7 -3.
// Used by adpv2_encoder_block for probe mean, delta, and K* computation.
//
// Instantiated as: u_div
// ============================================================
module sdiv_round32_seq (
  input  logic               clk,
  input  logic               rst_n,
  input  logic               start,
  input  logic signed [63:0] num,
  input  logic [31:0]        den,
  output logic               busy,
  output logic               done,
  output logic signed [31:0] q
);

  logic        sign_reg;
  logic [63:0] num_abs_reg;
  logic [31:0] den_reg;
  logic [64:0] rem_reg;
  logic [63:0] quot_reg;
  logic [6:0]  i_reg;

  logic [64:0] den_ext_w;
  logic        bit_in_w;
  logic [64:0] rem_shift_w;
  logic [64:0] rem_next_w;
  logic [63:0] quot_next_w;

  logic [64:0] rem2_w;
  logic [63:0] q_abs64_w;
  logic [63:0] q_round64_w;
  logic signed [63:0] q_signed64_w;

  always_comb begin
    den_ext_w   = {33'd0, den_reg};
    bit_in_w    = num_abs_reg[63 - i_reg];
    rem_shift_w = {rem_reg[63:0], bit_in_w};
    if (rem_shift_w >= den_ext_w) begin
      rem_next_w  = rem_shift_w - den_ext_w;
      quot_next_w = quot_reg | (64'd1 << (63 - i_reg));
    end else begin
      rem_next_w  = rem_shift_w;
      quot_next_w = quot_reg;
    end
    rem2_w       = rem_next_w << 1;
    q_abs64_w    = quot_next_w;
    q_round64_w  = (rem2_w >= den_ext_w) ?
                   (q_abs64_w + 64'd1) : q_abs64_w;
    q_signed64_w = sign_reg ?
                   -$signed(q_round64_w) : $signed(q_round64_w);
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sign_reg <= 1'b0; num_abs_reg <= '0; den_reg <= '0;
      rem_reg  <= '0;   quot_reg    <= '0; i_reg   <= '0;
      busy     <= 1'b0; done        <= 1'b0; q     <= '0;
    end else begin
      done <= 1'b0;
      if (start && !busy) begin
        if (den == '0) begin
          q <= '0; done <= 1'b1;
        end else begin
          sign_reg    <= (num < 0);
          num_abs_reg <= (num < 0) ? $unsigned(-num) : $unsigned(num);
          den_reg     <= den;
          rem_reg     <= '0; quot_reg <= '0; i_reg <= 7'd0;
          busy        <= 1'b1;
        end
      end else if (busy) begin
        rem_reg  <= rem_next_w;
        quot_reg <= quot_next_w;
        if (i_reg == 7'd63) begin
          q <= q_signed64_w[31:0]; busy <= 1'b0; done <= 1'b1;
        end else begin
          i_reg <= i_reg + 1;
        end
      end
    end
  end

endmodule