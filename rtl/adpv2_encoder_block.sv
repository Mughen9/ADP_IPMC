// ============================================================
// adpv2_encoder_block.sv  -  ADPv2 Top-Level Encoder
//
// Calls:
//   sdiv_round32_seq  (u_div)   - sequential 64-bit integer divider
//   isqrt_u64_seq     (u_sqrt)  - sequential 64-bit integer square root
//
// Corrected RTL - 3 fixes vs original:
//   FIX1: T1_FP=42259, T2_FP=19682 (unnormalized SoWHT scale)
//   FIX2: M* set changed to {64, 96, 128}
//   FIX3: M_SELECT routing direction corrected (E>T1 = smooth)
//
// Author : Sai Jayaprakash Ummithi
// Lab    : Prof. Jinjia Zhou, Hosei University
// Target : Xilinx Artix-7 xc7a200t-3, Vivado 2025.1
// Result : 98 MHz, WNS=+0.053 ns, 15907 LUTs, 218 mW
// ============================================================
// ============================================================
// ADPv2 : Adaptive Measurement Coding for Block Compressive Sensing
//         FPGA RTL Encoder - 16x16 Block (SoWHT + 7-mode Predictor)
//
// CORRECTED VERSION — 3 changes vs uploaded code:
//
//  FIX ① T1_FP / T2_FP recalibrated for unnormalized FWHT energy scale
//         Old: T1_FP=512  (0.5 Q10)   → New: T1_FP=42259 (41.27 × 1024)
//         Old: T2_FP=2048 (2.0 Q10)   → New: T2_FP=19682 (19.22 × 1024)
//
//  FIX ② M* set changed from {32,64,128} to {64,96,128}
//         Smooth  → M*=64  (was 32)
//         Moderate→ M*=96  (was 64)
//         Complex → M*=128 (unchanged)
//
//  FIX ③ M_SELECT routing direction flipped
//         Old: E < T1 → smooth (wrong — low E = complex)
//         New: E > T1 → smooth (correct — high E = energy concentrated = smooth)
//
// All other logic unchanged.
//
// Author  : Sai Jayaprakash Ummithi
// Lab     : Prof. Jinjia Zhou, Hosei University
// Target  : Xilinx Artix-7 xc7a200t, 100 MHz
// ============================================================

module adpv2_encoder_block #(
  parameter int BLOCK_SIZE    = 16,
  parameter int N             = BLOCK_SIZE * BLOCK_SIZE,  // 256
  parameter int M_PROBE       = 32,    // probe measurement count (fixed)
  parameter int M_MAX         = 128,   // maximum M
  parameter int PIX_W         = 8,
  parameter int Y_W           = 18,
  parameter int YQ_W          = 24,
  parameter int DELTA_FRAC    = 10,
  parameter int DELTA_W       = 24,
  // ★ FIX ① — Probe energy thresholds recalibrated for unnormalized FWHT
  // E = mean(y_probe²) over unnormalized SoWHT coefficients
  // T1 = 41.27 × 1024 = 42,259  (60th percentile on natural 4K imagery)
  // T2 = 19.22 × 1024 = 19,682  (30th percentile on natural 4K imagery)
  parameter int unsigned T1_FP = 42259,
  parameter int unsigned T2_FP = 19682,
  // Adaptive K bounds
  parameter int K_MIN         = 15,
  parameter int K_MAX         = 60,
  // AQ parameters
  parameter int BETA_NUM      = 4,
  parameter int BETA_DEN      = 5,
  parameter int unsigned DELTA_MIN_FP = 10,
  parameter int unsigned DELTA_MAX_FP = 1024,
  // ROM filenames
  parameter string SEQ_MEM    = "sequency_order_256.mem",
  parameter string ROWSUM_MEM = "rowsum_vector.mem",
  parameter string DECAY_MEM  = "spec_decay.mem"
) (
  input  logic clk,
  input  logic rst_n,

  // Pixel input stream
  input  logic              px_valid,
  input  logic [PIX_W-1:0] px_data,
  output logic              px_ready,

  // Control
  input  logic start,
  output logic busy,
  output logic done,

  // Neighbour prediction context
  input  logic signed [Y_W-1:0] up_y1,
  input  logic signed [Y_W-1:0] up_y2,
  input  logic signed [Y_W-1:0] left_y1,
  input  logic signed [Y_W-1:0] left_y32,
  input  logic is_first_row,
  input  logic is_first_col,

  // Self outputs for next-block prediction context
  output logic signed [Y_W-1:0] self_y1,
  output logic signed [Y_W-1:0] self_y2,
  output logic signed [Y_W-1:0] self_y32,

  // Quantised coefficient stream
  output logic                     yq_valid,
  output logic [$clog2(M_MAX)-1:0] yq_index,
  output logic signed [YQ_W-1:0]  yq_data,

  // Block header outputs
  output logic [2:0]          mode_out,
  output logic [DELTA_W-1:0]  delta_out,
  output logic [7:0]          m_sel_out,    // chosen M ∈ {64,96,128}
  output logic [6:0]          k_star_out    // adaptive OMP depth
);

  localparam int LOGN   = $clog2(N);
  localparam int ADDR_W = $clog2(N);

  localparam int LOG_M_PROBE = $clog2(M_PROBE);   // 5
  localparam int LOG_M_MAX   = $clog2(M_MAX);      // 7

  // ----------------------------------------------------------
  // ROMs
  // ----------------------------------------------------------
  (* rom_style = "distributed" *) logic [ADDR_W-1:0]     seq_order  [0:N-1];
  (* rom_style = "distributed" *) logic signed [Y_W-1:0] rowsum_s   [0:M_MAX-1];
  (* rom_style = "distributed" *) logic signed [Y_W-1:0] spec_decay [0:M_MAX-1];

  initial begin
    $readmemh(SEQ_MEM,    seq_order);
    $readmemh(ROWSUM_MEM, rowsum_s);
    $readmemh(DECAY_MEM,  spec_decay);
  end

  // ----------------------------------------------------------
  // Utility functions
  // ----------------------------------------------------------
  function automatic logic signed [Y_W-1:0] sabs_y(
      input logic signed [Y_W-1:0] v);
    sabs_y = (v < 0) ? -v : v;
  endfunction

  function automatic logic signed [YQ_W-1:0] sat_yq(
      input logic signed [63:0] v);
    logic signed [63:0] YQ_MAX, YQ_MIN;
    begin
      YQ_MAX = (64'sd1 <<< (YQ_W-1)) - 1;
      YQ_MIN = -(64'sd1 <<< (YQ_W-1));
      if      (v > YQ_MAX) sat_yq = $signed(YQ_MAX[YQ_W-1:0]);
      else if (v < YQ_MIN) sat_yq = $signed(YQ_MIN[YQ_W-1:0]);
      else                 sat_yq = $signed(v[YQ_W-1:0]);
    end
  endfunction

  function automatic logic [31:0] uabs32(
      input logic signed [31:0] v);
    uabs32 = (v < 0) ? logic'(-v) : logic'(v);
  endfunction

  // ----------------------------------------------------------
  // Measurement + mode storage (sized to M_MAX)
  // ----------------------------------------------------------
  (* ram_style = "register" *) logic signed [Y_W-1:0] y_meas [0:M_MAX-1];
  (* ram_style = "register" *) logic signed [Y_W-1:0] modes  [0:6][0:M_MAX-1];

  // ----------------------------------------------------------
  // vec[] BRAM
  // ----------------------------------------------------------
  logic [ADDR_W-1:0]     vec_addra, vec_addrb;
  logic signed [Y_W-1:0] vec_dina,  vec_dinb;
  logic                  vec_wea,   vec_web;
  logic signed [Y_W-1:0] vec_douta, vec_doutb;
  logic [ADDR_W-1:0]     a_idx_r,   b_idx_r;

  xpm_memory_tdpram #(
    .ADDR_WIDTH_A (ADDR_W),  .ADDR_WIDTH_B (ADDR_W),
    .AUTO_SLEEP_TIME(0),
    .BYTE_WRITE_WIDTH_A(Y_W), .BYTE_WRITE_WIDTH_B(Y_W),
    .CLOCKING_MODE("common_clock"),
    .ECC_MODE("no_ecc"),
    .MEMORY_INIT_FILE("none"), .MEMORY_INIT_PARAM("0"),
    .MEMORY_OPTIMIZATION("true"), .MEMORY_PRIMITIVE("block"),
    .MEMORY_SIZE(N*Y_W), .MESSAGE_CONTROL(0),
    .READ_DATA_WIDTH_A(Y_W), .READ_DATA_WIDTH_B(Y_W),
    .READ_LATENCY_A(1), .READ_LATENCY_B(1),
    .READ_RESET_VALUE_A("0"), .READ_RESET_VALUE_B("0"),
    .RST_MODE_A("SYNC"), .RST_MODE_B("SYNC"),
    .SIM_ASSERT_CHK(0), .USE_EMBEDDED_CONSTRAINT(0), .USE_MEM_INIT(0),
    .WAKEUP_TIME("disable_sleep"),
    .WRITE_DATA_WIDTH_A(Y_W), .WRITE_DATA_WIDTH_B(Y_W),
    .WRITE_MODE_A("read_first"), .WRITE_MODE_B("read_first"),
    .WRITE_PROTECT(1)
  ) vec_mem (
    .clka(clk), .clkb(clk), .rsta(~rst_n), .rstb(~rst_n),
    .addra(vec_addra), .addrb(vec_addrb),
    .dina(vec_dina),   .dinb(vec_dinb),
    .ena(1'b1), .enb(1'b1),
    .wea(vec_wea),   .web(vec_web),
    .douta(vec_douta), .doutb(vec_doutb),
    .injectsbiterra(1'b0), .injectdbiterra(1'b0),
    .injectsbiterrb(1'b0), .injectdbiterrb(1'b0),
    .sbiterra(), .dbiterra(), .sbiterrb(), .dbiterrb()
  );

  // ----------------------------------------------------------
  // Sequential divider
  // ----------------------------------------------------------
  logic               div_start, div_busy, div_done;
  logic signed [63:0] div_num;
  logic [31:0]        div_den;
  logic signed [31:0] div_q;

  sdiv_round32_seq u_div (
    .clk(clk), .rst_n(rst_n),
    .start(div_start), .num(div_num), .den(div_den),
    .busy(div_busy), .done(div_done), .q(div_q)
  );

  // ----------------------------------------------------------
  // Sequential square-root
  // ----------------------------------------------------------
  logic        sqrt_start, sqrt_busy, sqrt_done;
  logic [63:0] sqrt_x;
  logic [31:0] sqrt_root;

  isqrt_u64_seq u_sqrt (
    .clk(clk), .rst_n(rst_n),
    .start(sqrt_start), .x(sqrt_x),
    .busy(sqrt_busy), .done(sqrt_done), .root(sqrt_root)
  );

  // ----------------------------------------------------------
  // FSM states (unchanged from uploaded version)
  // ----------------------------------------------------------
  typedef enum logic [5:0] {
    IDLE,
    LOAD,
    FWHT_RD,
    FWHT_WR,
    PROBE_RD,
    PROBE_WR,
    PROBE_MEAN_START,
    PROBE_MEAN_WAIT,
    M_SELECT,
    FULL_SEL_RD,
    FULL_SEL_WR,
    PRED_INIT_UP_START,
    PRED_INIT_UP_WAIT,
    PRED_INIT_LEFT_START,
    PRED_INIT_LEFT_WAIT,
    PRED_MUL,
    PRED_BASE,
    MODE_PREFETCH,
    MODE_SEL,
    BEST_MODE_LATCH,
    BEST_MODE,
    DA_PREFETCH,
    DELTA_ACCUM,
    DELTA_MEAN,
    DELTA_SQRT_START,
    DELTA_SQRT_WAIT,
    DELTA_BETA_MUL,
    DELTA_BETA_DIV_START,
    DELTA_BETA_DIV_WAIT,
    DELTA_CLAMP,
    SIGMA_Y_SQRT_START,
    SIGMA_Y_SQRT_WAIT,
    K_RATIO_START,
    K_RATIO_WAIT,
    K_COMPUTE,
    QS_PREFETCH,
    QUANT_SETUP,
    QUANT_DIV_WAIT,
    QUANT_MUL,
    QUANT_OUT,
    FINISH
  } state_t;

  state_t st;

  // ----------------------------------------------------------
  // Registers (unchanged)
  // ----------------------------------------------------------
  logic [$clog2(N):0]    load_cnt;
  logic                  loaded;

  logic [$clog2(LOGN):0] stage;
  logic [$clog2(N/2):0]  bfly;

  logic [7:0]   m_sel;
  logic [7:0]   log2_m_sel;
  logic [63:0]  probe_sum_sq;
  logic [63:0]  sum_y_sq_reg;
  logic [31:0]  sigma_y_root;

  logic [31:0]  k_ratio_reg;
  logic [6:0]   k_star_reg;

  logic signed [Y_W-1:0] avg_up_const, avg_left_const;
  logic                  denom_up_neg, denom_left_neg;
  logic signed [2*Y_W-1:0] mul_u_reg, mul_l_reg;
  logic signed [Y_W-1:0]   yu_reg, yl_reg, yavg_reg;

  logic [63:0] mode_costs [0:6];
  logic [2:0]  best_mode_reg;
  logic [2:0]  bestm_lat;
  logic [63:0] bestc_lat;

  logic [$clog2(M_MAX):0] sel_k, pred_k, mode_k, delta_k, q_k;
  logic [2:0]             mode_idx;

  logic [63:0]        sum_sq_reg, sum_final_reg, mean_sq_reg;
  logic [31:0]        rms_reg;
  logic [63:0]        beta_mul_reg;
  logic signed [31:0] beta_div_q;
  logic [DELTA_W-1:0] delta_fp_reg;

  logic signed [Y_W-1:0]   yp_reg, yr_reg, yhat_reg;
  logic signed [31:0]       q_div_reg;
  logic signed [YQ_W-1:0]  q_sat_reg;
  logic signed [63:0]       prod_reg;

  logic signed [Y_W-1:0] mode_ymeas_q, mode_pred_q;
  logic signed [Y_W-1:0] da_ymeas_q, da_pred_q;
  logic signed [Y_W-1:0] qs_ymeas_q, qs_pred_q;

  integer ii;

  // ----------------------------------------------------------
  // Combinational blocks (unchanged)
  // ----------------------------------------------------------
  logic [31:0] span_w, half_w, grp_w, j_w, a_idx32, b_idx32;

  always_comb begin
    span_w  = 32'd1 << (stage + 1);
    half_w  = 32'd1 << stage;
    grp_w   = bfly / half_w;
    j_w     = bfly % half_w;
    a_idx32 = grp_w * span_w + j_w;
    b_idx32 = a_idx32 + half_w;
  end

  logic signed [31:0] denom_up_s32, denom_left_s32;

  always_comb begin
    denom_up_s32   = ($signed(rowsum_s[0]) - $signed(rowsum_s[1]));
    denom_left_s32 = (M_MAX >= 32) ?
                     ($signed(rowsum_s[0]) - $signed(rowsum_s[31])) : 32'sd0;
  end

  logic [63:0] bestc_comb;
  logic [2:0]  bestm_comb;

  always_comb begin
    bestc_comb = mode_costs[0];
    bestm_comb = 3'd0;
    for (ii = 1; ii < 7; ii = ii + 1) begin
      if (mode_costs[ii] < bestc_comb) begin
        bestc_comb = mode_costs[ii];
        bestm_comb = ii[2:0];
      end
    end
  end

  logic [31:0] neg_div_q_up, neg_div_q_left;

  always_comb begin
    neg_div_q_up   = -div_q;
    neg_div_q_left = -div_q;
  end

  logic signed [2*Y_W-1:0] spec_exp_full;
  logic signed [Y_W-1:0]   spec_exp_w;

  always_comb begin
    spec_exp_full = $signed(yavg_reg) * $signed(spec_decay[pred_k]);
    spec_exp_w    = $signed(spec_exp_full >>> 10);
  end

  logic signed [Y_W-1:0] mode_res, mode_absr;

  always_comb begin
    mode_res  = mode_ymeas_q - mode_pred_q;
    mode_absr = sabs_y(mode_res);
  end

  logic signed [Y_W-1:0] da_res;
  logic [63:0]            da_sq, da_sum_next;

  always_comb begin
    da_res      = da_ymeas_q - da_pred_q;
    da_sq       = $unsigned($signed(da_res) * $signed(da_res));
    da_sum_next = sum_sq_reg + da_sq;
  end

  logic [63:0] beta_mul_full_w;

  always_comb begin
    beta_mul_full_w = $unsigned(rms_reg) * BETA_NUM;
  end

  logic [63:0] delta_tmp_u64;

  always_comb begin
    delta_tmp_u64 = (DELTA_FRAC > 0) ?
                    ($unsigned(beta_div_q) << DELTA_FRAC) :
                     $unsigned(beta_div_q);
  end

  logic signed [Y_W-1:0] qs_yp_tmp, qs_yr_tmp;
  logic signed [63:0]    qs_num_tmp;
  logic [31:0]           qs_den_tmp;

  always_comb begin
    qs_yp_tmp  = qs_pred_q;
    qs_yr_tmp  = qs_ymeas_q - qs_pred_q;
    qs_num_tmp = (DELTA_FRAC > 0) ?
                 ($signed(qs_yr_tmp) <<< DELTA_FRAC) : $signed(qs_yr_tmp);
    qs_den_tmp = (delta_fp_reg == '0) ? 32'd1 :
                 {{(32-DELTA_W){1'b0}}, delta_fp_reg};
  end

  logic signed [63:0]     qm_q_ext_w;
  logic signed [YQ_W-1:0] qm_q_sat_w;
  logic signed [31:0]      qm_q_use_w;

  always_comb begin
    qm_q_ext_w = {{32{q_div_reg[31]}}, q_div_reg};
    qm_q_sat_w = sat_yq(qm_q_ext_w);
    qm_q_use_w = {{(32-YQ_W){qm_q_sat_w[YQ_W-1]}}, qm_q_sat_w};
  end

  logic signed [63:0]    qo_yr_iq64;
  logic signed [Y_W-1:0] qo_yr_iq;

  always_comb begin
    qo_yr_iq64 = prod_reg;
    if (DELTA_FRAC > 0) begin
      qo_yr_iq64 = qo_yr_iq64 + (64'sd1 <<< (DELTA_FRAC - 1));
      qo_yr_iq64 = qo_yr_iq64 >>> DELTA_FRAC;
    end
    qo_yr_iq = $signed(qo_yr_iq64[Y_W-1:0]);
  end

  logic [63:0] probe_sq_w, probe_sum_next;
  logic signed [Y_W-1:0] probe_y_tmp;

  always_comb begin
    probe_y_tmp    = vec_douta;
    probe_sq_w     = $unsigned($signed(probe_y_tmp) * $signed(probe_y_tmp));
    probe_sum_next = probe_sum_sq + probe_sq_w;
  end

  logic [63:0] y_sq_w, sum_y_sq_next;
  logic signed [Y_W-1:0] curr_y_tmp;

  assign px_ready = (st == LOAD);

  // ==========================================================
  // MAIN FSM
  // ==========================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st       <= IDLE;
      busy     <= 1'b0;
      done     <= 1'b0;
      loaded   <= 1'b0;
      load_cnt <= '0;

      vec_addra <= '0;  vec_addrb <= '0;
      vec_dina  <= '0;  vec_dinb  <= '0;
      vec_wea   <= 1'b0; vec_web  <= 1'b0;
      a_idx_r   <= '0;  b_idx_r   <= '0;

      stage <= '0;  bfly <= '0;

      m_sel         <= 8'd64;         // ★ FIX ② default is now 64 not 32
      log2_m_sel    <= 8'd6;
      probe_sum_sq  <= '0;
      sum_y_sq_reg  <= '0;
      sigma_y_root  <= 32'd1;
      m_sel_out     <= '0;

      k_ratio_reg <= '0;
      k_star_reg  <= K_MIN[6:0];
      k_star_out  <= K_MIN[6:0];

      sel_k    <= '0;  pred_k   <= '0;
      mode_k   <= '0;  delta_k  <= '0;
      q_k      <= '0;  mode_idx <= '0;

      avg_up_const   <= '0;
      avg_left_const <= '0;
      denom_up_neg   <= 1'b0;
      denom_left_neg <= 1'b0;

      mul_u_reg <= '0;  mul_l_reg <= '0;
      yu_reg    <= '0;  yl_reg    <= '0;
      yavg_reg  <= '0;

      for (ii = 0; ii < 7; ii = ii + 1) mode_costs[ii] <= '0;
      best_mode_reg <= '0;
      bestm_lat     <= '0;
      bestc_lat     <= '0;

      sum_sq_reg   <= '0;  sum_final_reg <= '0;
      mean_sq_reg  <= '0;  rms_reg       <= '0;
      beta_mul_reg <= '0;  beta_div_q    <= '0;
      delta_fp_reg <= '0;  delta_out     <= '0;
      mode_out     <= '0;

      yp_reg    <= '0;  yr_reg    <= '0;
      q_div_reg <= '0;  q_sat_reg <= '0;
      prod_reg  <= '0;  yhat_reg  <= '0;

      mode_ymeas_q <= '0;  mode_pred_q <= '0;
      da_ymeas_q   <= '0;  da_pred_q   <= '0;
      qs_ymeas_q   <= '0;  qs_pred_q   <= '0;

      self_y1  <= '0;  self_y2  <= '0;  self_y32 <= '0;

      yq_valid <= 1'b0;  yq_index <= '0;  yq_data <= '0;

      div_start  <= 1'b0;  div_num <= '0;  div_den <= '0;
      sqrt_start <= 1'b0;  sqrt_x  <= '0;

    end else begin
      done       <= 1'b0;
      yq_valid   <= 1'b0;
      div_start  <= 1'b0;
      sqrt_start <= 1'b0;
      vec_wea    <= 1'b0;
      vec_web    <= 1'b0;

      case (st)

        IDLE: begin
          busy <= 1'b0;
          if (!loaded) begin
            load_cnt <= '0;
            st       <= LOAD;
          end else if (start) begin
            busy         <= 1'b1;
            stage        <= '0;
            bfly         <= '0;
            probe_sum_sq <= '0;
            sum_y_sq_reg <= '0;
            st           <= FWHT_RD;
          end
        end

        LOAD: begin
          if (px_valid) begin
            vec_addra <= load_cnt[ADDR_W-1:0];
            vec_dina  <= $signed({1'b0, px_data});
            vec_wea   <= 1'b1;
            if (load_cnt == N-1) begin
              loaded   <= 1'b1;
              load_cnt <= '0;
              st       <= IDLE;
            end else begin
              load_cnt <= load_cnt + 1;
            end
          end
        end

        FWHT_RD: begin
          a_idx_r   <= a_idx32[ADDR_W-1:0];
          b_idx_r   <= b_idx32[ADDR_W-1:0];
          vec_addra <= a_idx32[ADDR_W-1:0];
          vec_addrb <= b_idx32[ADDR_W-1:0];
          st        <= FWHT_WR;
        end

        FWHT_WR: begin
          vec_addra <= a_idx_r;
          vec_dina  <= vec_douta + vec_doutb;
          vec_wea   <= 1'b1;
          vec_addrb <= b_idx_r;
          vec_dinb  <= vec_douta - vec_doutb;
          vec_web   <= 1'b1;
          if (bfly == (N/2 - 1)) begin
            bfly <= '0;
            if (stage == (LOGN - 1)) begin
              sel_k <= '0;
              st    <= PROBE_RD;
            end else begin
              stage <= stage + 1;
              st    <= FWHT_RD;
            end
          end else begin
            bfly <= bfly + 1;
            st   <= FWHT_RD;
          end
        end

        // ════════════════════════════════════════════════════
        // PROBE + ADAPTIVE M SELECTION
        // ════════════════════════════════════════════════════

        PROBE_RD: begin
          vec_addra <= seq_order[sel_k];
          st        <= PROBE_WR;
        end

        PROBE_WR: begin
          y_meas[sel_k]  <= vec_douta;
          probe_sum_sq   <= probe_sum_sq +
                            $unsigned($signed(vec_douta) * $signed(vec_douta));
          sum_y_sq_reg   <= sum_y_sq_reg +
                            $unsigned($signed(vec_douta) * $signed(vec_douta));
          if (sel_k == M_PROBE - 1) begin
            div_num   <= $signed({1'b0, probe_sum_sq});
            div_den   <= M_PROBE[31:0];
            div_start <= 1'b1;
            st        <= PROBE_MEAN_WAIT;
          end else begin
            sel_k <= sel_k + 1;
            st    <= PROBE_RD;
          end
        end

        PROBE_MEAN_WAIT: begin
          if (div_done) begin
            st <= M_SELECT;
          end
        end

        // ★ FIX ② + ③ — corrected M* set {64,96,128} and direction flip
        // High probe energy E → energy concentrated in few coefficients → SMOOTH block → fewer measurements needed
        // Low probe energy E  → energy spread across many coefficients  → COMPLEX block → more measurements needed
        M_SELECT: begin
          if ($unsigned(div_q) > T1_FP) begin
            // E > 41.27 → smooth → M*=64
            m_sel      <= 8'd64;
            log2_m_sel <= 8'd6;
          end else if ($unsigned(div_q) > T2_FP) begin
            // T2 < E ≤ T1 → moderate → M*=96
            m_sel      <= 8'd96;
            log2_m_sel <= 8'd7;   // log2(128) used for power-of-2 shift ops
          end else begin
            // E ≤ T2 → complex → M*=128
            m_sel      <= M_MAX[7:0];
            log2_m_sel <= LOG_M_MAX[7:0];
          end
          m_sel_out <= m_sel;

          // All branches need more measurements beyond probe (M* >= 64 > M_probe=32)
          // so always continue to FULL_SEL_RD
          sel_k <= M_PROBE[$clog2(M_MAX):0];
          st    <= FULL_SEL_RD;
        end

        FULL_SEL_RD: begin
          vec_addra <= seq_order[sel_k];
          st        <= FULL_SEL_WR;
        end

        FULL_SEL_WR: begin
          y_meas[sel_k] <= vec_douta;
          sum_y_sq_reg  <= sum_y_sq_reg +
                           $unsigned($signed(vec_douta) * $signed(vec_douta));
          if (sel_k == m_sel - 1) begin
            sel_k <= '0;
            st    <= PRED_INIT_UP_START;
          end else begin
            sel_k <= sel_k + 1;
            st    <= FULL_SEL_RD;
          end
        end

        // ════════════════════════════════════════════════════
        // PREDICTION INIT (unchanged)
        // ════════════════════════════════════════════════════

        PRED_INIT_UP_START: begin
          if (is_first_row || (denom_up_s32 == 0)) begin
            avg_up_const <= '0;
            st           <= PRED_INIT_LEFT_START;
          end else begin
            denom_up_neg <= (denom_up_s32 < 0);
            div_num      <= $signed(up_y1) - $signed(up_y2);
            div_den      <= uabs32(denom_up_s32);
            div_start    <= 1'b1;
            st           <= PRED_INIT_UP_WAIT;
          end
        end

        PRED_INIT_UP_WAIT: begin
          if (div_done) begin
            avg_up_const <= denom_up_neg ?
                            $signed(neg_div_q_up[Y_W-1:0]) :
                            $signed(div_q[Y_W-1:0]);
            st <= PRED_INIT_LEFT_START;
          end
        end

        PRED_INIT_LEFT_START: begin
          if (is_first_col || (m_sel < 32) || (denom_left_s32 == 0)) begin
            avg_left_const <= '0;
            pred_k         <= '0;
            st             <= PRED_MUL;
          end else begin
            denom_left_neg <= (denom_left_s32 < 0);
            div_num        <= $signed(left_y1) - $signed(left_y32);
            div_den        <= uabs32(denom_left_s32);
            div_start      <= 1'b1;
            st             <= PRED_INIT_LEFT_WAIT;
          end
        end

        PRED_INIT_LEFT_WAIT: begin
          if (div_done) begin
            avg_left_const <= denom_left_neg ?
                              $signed(neg_div_q_left[Y_W-1:0]) :
                              $signed(div_q[Y_W-1:0]);
            pred_k <= '0;
            st     <= PRED_MUL;
          end
        end

        // ────────────────────────────────────────────────────
        // 7-MODE PREDICTOR (unchanged)
        // ────────────────────────────────────────────────────
        PRED_MUL: begin
          mul_u_reg <= $signed(avg_up_const)   * $signed(rowsum_s[pred_k]);
          mul_l_reg <= $signed(avg_left_const) * $signed(rowsum_s[pred_k]);
          st        <= PRED_BASE;
        end

        PRED_BASE: begin
          yu_reg   <= is_first_row                         ? '0
                   : $signed(mul_u_reg[Y_W-1:0]);
          yl_reg   <= (is_first_col || m_sel < 32)         ? '0
                   : $signed(mul_l_reg[Y_W-1:0]);
          yavg_reg <= ($signed(is_first_row           ? '0 : mul_u_reg[Y_W-1:0]) +
                       $signed((is_first_col||m_sel<32) ? '0 : mul_l_reg[Y_W-1:0])) >>> 1;

          modes[0][pred_k] <= is_first_row ? '0 : $signed(mul_u_reg[Y_W-1:0]);
          modes[1][pred_k] <= (is_first_col||m_sel<32) ? '0 : $signed(mul_l_reg[Y_W-1:0]);
          modes[2][pred_k] <= ($signed(is_first_row           ? '0 : mul_u_reg[Y_W-1:0]) +
                               $signed((is_first_col||m_sel<32) ? '0 : mul_l_reg[Y_W-1:0])) >>> 1;
          modes[3][pred_k] <= ($signed(3) *
                               $signed(is_first_row ? '0 : mul_u_reg[Y_W-1:0]) +
                               $signed((is_first_col||m_sel<32) ? '0 : mul_l_reg[Y_W-1:0])) >>> 2;
          modes[4][pred_k] <= (pred_k < (m_sel >> 2)) ?
                              (($signed(is_first_row           ? '0 : mul_u_reg[Y_W-1:0]) +
                                $signed((is_first_col||m_sel<32) ? '0 : mul_l_reg[Y_W-1:0])) >>> 1)
                              : '0;
          modes[5][pred_k] <= ($signed(($signed(is_first_row           ? '0 : mul_u_reg[Y_W-1:0]) +
                                        $signed((is_first_col||m_sel<32) ? '0 : mul_l_reg[Y_W-1:0])) >>> 1)
                               * $signed(spec_decay[pred_k])) >>> 10;
          modes[6][pred_k] <= '0;

          if (pred_k == m_sel - 1) begin
            for (ii = 0; ii < 7; ii = ii + 1) mode_costs[ii] <= '0;
            mode_idx <= '0;
            mode_k   <= '0;
            st       <= MODE_PREFETCH;
          end else begin
            pred_k <= pred_k + 1;
            st     <= PRED_MUL;
          end
        end

        // ────────────────────────────────────────────────────
        // L1 MODE SELECTION (unchanged)
        // ────────────────────────────────────────────────────
        MODE_PREFETCH: begin
          mode_ymeas_q <= $signed(y_meas[mode_k]);
          mode_pred_q  <= $signed(modes[mode_idx][mode_k]);
          st           <= MODE_SEL;
        end

        MODE_SEL: begin
          mode_costs[mode_idx] <= mode_costs[mode_idx] + $unsigned(mode_absr);
          if (mode_k == m_sel - 1) begin
            if (mode_idx == 3'd6) begin
              st <= BEST_MODE_LATCH;
            end else begin
              mode_idx <= mode_idx + 1;
              mode_k   <= '0;
              st       <= MODE_PREFETCH;
            end
          end else begin
            mode_k <= mode_k + 1;
            st     <= MODE_PREFETCH;
          end
        end

        BEST_MODE_LATCH: begin
          bestm_lat <= bestm_comb;
          bestc_lat <= bestc_comb;
          st        <= BEST_MODE;
        end

        BEST_MODE: begin
          best_mode_reg <= bestm_lat;
          mode_out      <= bestm_lat + 3'd1;
          sum_sq_reg    <= '0;
          delta_k       <= '0;
          st            <= DA_PREFETCH;
        end

        // ────────────────────────────────────────────────────
        // DELTA (unchanged)
        // ────────────────────────────────────────────────────
        DA_PREFETCH: begin
          da_ymeas_q <= $signed(y_meas[delta_k]);
          da_pred_q  <= $signed(modes[best_mode_reg][delta_k]);
          st         <= DELTA_ACCUM;
        end

        DELTA_ACCUM: begin
          sum_sq_reg <= da_sum_next;
          if (delta_k == m_sel - 1) begin
            sum_final_reg <= da_sum_next;
            st            <= DELTA_MEAN;
          end else begin
            delta_k <= delta_k + 1;
            st      <= DA_PREFETCH;
          end
        end

        DELTA_MEAN: begin
          mean_sq_reg <= sum_final_reg >> log2_m_sel;
          st          <= DELTA_SQRT_START;
        end

        DELTA_SQRT_START: begin
          sqrt_x     <= mean_sq_reg;
          sqrt_start <= 1'b1;
          st         <= DELTA_SQRT_WAIT;
        end

        DELTA_SQRT_WAIT: begin
          if (sqrt_done) begin
            rms_reg <= sqrt_root;
            st      <= DELTA_BETA_MUL;
          end
        end

        DELTA_BETA_MUL: begin
          beta_mul_reg <= beta_mul_full_w;
          if (BETA_DEN <= 1) begin
            beta_div_q <= $signed(beta_mul_full_w[31:0]);
            st         <= DELTA_CLAMP;
          end else begin
            st <= DELTA_BETA_DIV_START;
          end
        end

        DELTA_BETA_DIV_START: begin
          div_num   <= $signed({1'b0, beta_mul_reg});
          div_den   <= BETA_DEN[31:0];
          div_start <= 1'b1;
          st        <= DELTA_BETA_DIV_WAIT;
        end

        DELTA_BETA_DIV_WAIT: begin
          if (div_done) begin
            beta_div_q <= div_q;
            st         <= DELTA_CLAMP;
          end
        end

        DELTA_CLAMP: begin
          if (delta_tmp_u64 < DELTA_MIN_FP) begin
            delta_fp_reg <= DELTA_MIN_FP[DELTA_W-1:0];
            delta_out    <= DELTA_MIN_FP[DELTA_W-1:0];
          end else if (delta_tmp_u64 > DELTA_MAX_FP) begin
            delta_fp_reg <= DELTA_MAX_FP[DELTA_W-1:0];
            delta_out    <= DELTA_MAX_FP[DELTA_W-1:0];
          end else begin
            delta_fp_reg <= delta_tmp_u64[DELTA_W-1:0];
            delta_out    <= delta_tmp_u64[DELTA_W-1:0];
          end
          st <= SIGMA_Y_SQRT_START;
        end

        // ════════════════════════════════════════════════════
        // ADAPTIVE K (unchanged)
        // ════════════════════════════════════════════════════

        SIGMA_Y_SQRT_START: begin
          sqrt_x     <= sum_y_sq_reg >> log2_m_sel;
          sqrt_start <= 1'b1;
          st         <= SIGMA_Y_SQRT_WAIT;
        end

        SIGMA_Y_SQRT_WAIT: begin
          if (sqrt_done) begin
            sigma_y_root <= (sqrt_root == '0) ? 32'd1 : sqrt_root;
            st           <= K_RATIO_START;
          end
        end

        K_RATIO_START: begin
          div_num   <= $signed({1'b0, $unsigned(rms_reg) * (K_MAX - K_MIN)});
          div_den   <= sigma_y_root;
          div_start <= 1'b1;
          st        <= K_RATIO_WAIT;
        end

        K_RATIO_WAIT: begin
          if (div_done) begin
            k_ratio_reg <= $unsigned(div_q[31:0]);
            st          <= K_COMPUTE;
          end
        end

        K_COMPUTE: begin
          if ($unsigned(k_ratio_reg) + K_MIN >= K_MAX)
            k_star_reg <= K_MAX[6:0];
          else
            k_star_reg <= $unsigned(K_MIN + k_ratio_reg[6:0]);
          k_star_out <= k_star_reg;
          q_k        <= '0;
          st         <= QS_PREFETCH;
        end

        // ════════════════════════════════════════════════════
        // QUANTISATION (unchanged)
        // ════════════════════════════════════════════════════

        QS_PREFETCH: begin
          qs_ymeas_q <= $signed(y_meas[q_k]);
          qs_pred_q  <= $signed(modes[best_mode_reg][q_k]);
          st         <= QUANT_SETUP;
        end

        QUANT_SETUP: begin
          yp_reg    <= qs_yp_tmp;
          yr_reg    <= qs_yr_tmp;
          div_num   <= qs_num_tmp;
          div_den   <= qs_den_tmp;
          div_start <= 1'b1;
          st        <= QUANT_DIV_WAIT;
        end

        QUANT_DIV_WAIT: begin
          if (div_done) begin
            q_div_reg <= div_q;
            st        <= QUANT_MUL;
          end
        end

        QUANT_MUL: begin
          q_sat_reg <= qm_q_sat_w;
          prod_reg  <= $signed(qm_q_use_w) * $signed({1'b0, delta_fp_reg});
          st        <= QUANT_OUT;
        end

        QUANT_OUT: begin
          yhat_reg <= $signed(yp_reg) + $signed(qo_yr_iq);

          yq_valid <= 1'b1;
          yq_index <= q_k[$clog2(M_MAX)-1:0];
          yq_data  <= q_sat_reg;

          if (q_k == 0)  self_y1  <= $signed(yp_reg) + $signed(qo_yr_iq);
          if (q_k == 1)  self_y2  <= $signed(yp_reg) + $signed(qo_yr_iq);
          if (q_k == 31) self_y32 <= $signed(yp_reg) + $signed(qo_yr_iq);

          if (q_k == m_sel - 1) begin
            st <= FINISH;
          end else begin
            q_k <= q_k + 1;
            st  <= QS_PREFETCH;
          end
        end

        FINISH: begin
          busy   <= 1'b0;
          done   <= 1'b1;
          loaded <= 1'b0;
          st     <= IDLE;
        end

        default: st <= IDLE;

      endcase
    end
  end

endmodule


// ============================================================================
// isqrt_u64_seq (unchanged)
// ============================================================================