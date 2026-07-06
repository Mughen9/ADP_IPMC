# ADP_IPMC — ADPv2 Adaptive Block Compressive Sensing Encoder

**Resolving the Sensing Paradox: Compressive-Domain Block Classification for Adaptive BCS with FPGA Validation**

Sai Jayaprakash Ummithi, Zhidi Yao, Jinjia Zhou  
Zhou Laboratory, Hosei University, Tokyo, Japan  
M.Eng Applied Informatics, September 2026

---

## Publication

> S. J. Ummithi, Z. Yao, and J. Zhou, "Resolving the Sensing Paradox: Compressive-Domain Block Classification for Adaptive BCS with FPGA Validation," *IEEE ICIP 2026 Workshop*, Tampere, Finland, Sep. 2026.

---

## What This Is

ADPv2 is an adaptive block compressive sensing (BCS) encoder that resolves the **sensing paradox** — the contradiction that every existing adaptive-M scheme classifies block complexity from pixel-domain statistics, which the sensor was architecturally designed never to read in full.

ADPv2 resolves this by using the energy of the first 32 WHT coefficients, produced as a byproduct of sensing itself, to classify each block into M* ∈ {64, 96, 128} measurements — with zero pixel-domain access beyond the initial projection.

---

## Key Results

| Sequence | IPMC-4 PSNR | ADPv2 PSNR | ΔPSNR | BD-Rate |
|----------|-------------|------------|-------|---------|
| Beauty | 38.90 dB | 39.13 dB | +0.52 dB | −12.0% |
| Bosphorus | 36.47 dB | 38.14 dB | +2.84 dB | −43.8% |
| HoneyBee | 35.64 dB | 36.47 dB | +0.68 dB | −33.9% |
| **Average** | 36.67 dB | **37.91 dB** | **+1.35 dB** | **−29.9%** |

*Qb=4, SR=0.25, UVG 4K dataset, Frame010. IPMC-4 values from Peetakul 2022 Table 1.*

---

## FPGA Implementation

| Resource | Used | Available | Util |
|----------|------|-----------|------|
| LUT6 | 15,907 | 134,600 | 11.82% |
| Flip-Flop | 15,796 | 269,200 | 5.87% |
| DSP48E1 | 6 | 740 | 0.81% |
| RAMB18E1 | 1 | 365 | 0.27% |
| Clock | 98 MHz | — | — |
| WNS | +0.053 ns | — | — |
| Power | 218 mW | — | — |

Target: Xilinx Artix-7 xc7a200t (−3), Vivado 2025.1, default strategy.  
RTL verified bit-exact against MATLAB golden model across 24,120 blocks, zero mismatches.

---

## Repository Structure

```
ADP_IPMC/
│
├── rtl/
│   └── adpv2_encoder_block.sv    # Top-level SystemVerilog RTL
│
├── constraints/
│   └── adpv2_timing.xdc          # Timing constraints (98 MHz, Artix-7)
│
├── sim/
│   └── adpv2_tb.sv               # Testbench stub
│
├── matlab/
│   ├── adpv2_main.m              # Entry point — run this
│   ├── adpv2_run.m               # ADPv2 encoder (verified core)
│   ├── ipmc4_run.m               # IPMC-4 baseline (comparison only)
│   ├── modes7.m                  # 7-mode spectral predictor
│   ├── get_delta.m               # Adaptive quantization step
│   ├── omp_solve.m               # OMP reconstruction
│   ├── psnr_calc.m               # PSNR utility
│   └── plot_figures.m            # Figure generator
│
└── README.md
```

---

## How To Run MATLAB

1. Open MATLAB
2. Set `IMG_DIR` in `adpv2_main.m` to your image folder
3. Place UVG 4K frames in that folder:
   - `Beauty_1920x1080_120fps_420_8bit_YUV_Frame010_Y.png`
   - `Bosphorus_1920x1080_120fps_420_8bit_YUV_Frame010_Y.png`
   - `HoneyBee_Frame1.png`
   - `ReadySteadyGo_1920x1080_120fps_420_8bit_YUV_Frame010_Y.png`
4. Run `>> adpv2_main`

A sanity check verifies results against thesis Table 4.1 before saving any figures. If any value differs by more than 0.10 dB the script warns and exits.

---

## How To Run FPGA

1. Open Vivado 2025.1
2. Create new project → Artix-7 xc7a200t-3
3. Add `rtl/adpv2_encoder_block.sv` as design source
4. Add `constraints/adpv2_timing.xdc` as constraint
5. Run Synthesis → Implementation → Generate Bitstream

---

## ADPv2 System Parameters

| Parameter | Value |
|-----------|-------|
| Block size | 16×16 (N=256) |
| M* set | {64, 96, 128} |
| Probe size | M_probe = 32 |
| T1 threshold | 41.27 (Q10: 42,259) |
| T2 threshold | 19.22 (Q10: 19,682) |
| β (quantization) | 4/5 |
| K* range | [15, 60] |
| Arithmetic | 18-bit Q8.8 signed fixed-point |

---

## License

MIT License — see LICENSE file.

---

## Contact

Sai Jayaprakash Ummithi  
Zhou Laboratory, Hosei University  
M.Eng Applied Informatics, Student ID: 24R8111
