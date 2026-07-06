% =========================================================================
%  adpv2_main.m  -  ADPv2 main entry point
%
%  Run this file only. All other files are called automatically.
%
%  Verified: Bosphorus Qb=4 gives ADPv2=38.14 dB, IPMC-4=35.30 dB
%  matching thesis Table 4.1 within 0.002 dB (2026-06-19)
%
%  Usage:
%    1. Set IMG_DIR to your image folder
%    2. >> adpv2_main
% =========================================================================
clear; clc; close all;

% ── Paths ────────────────────────────────────────────────────────────────
IMG_DIR = 'C:\Users\saira\OneDrive\Documents\MATLAB\Research\';
OUT_DIR = [IMG_DIR 'figs_combined\'];
if ~exist(OUT_DIR, 'dir'), mkdir(OUT_DIR); end

% ── Datasets ─────────────────────────────────────────────────────────────
DATASETS = {
    'Beauty_1920x1080_120fps_420_8bit_YUV_Frame010_Y.png',        'Beauty';
    'Bosphorus_1920x1080_120fps_420_8bit_YUV_Frame010_Y.png',     'Bosphorus';
    'HoneyBee_Frame1.png',                                         'HoneyBee';
    'ReadySteadyGo_1920x1080_120fps_420_8bit_YUV_Frame010_Y.png', 'RSG';
};

% ── Sanity check reference (thesis Table 4.1, Qb=4) ─────────────────────
THESIS_REF = struct( ...
    'Beauty',    struct('adp', 39.13, 'ip4', 38.60), ...
    'Bosphorus', struct('adp', 38.14, 'ip4', 35.30), ...
    'HoneyBee',  struct('adp', 36.47, 'ip4', 35.79), ...
    'RSG',       struct('adp', 32.10, 'ip4', 31.00)  ...
);
TOL_DB = 0.10;

% ── ADPv2 parameters ─────────────────────────────────────────────────────
bsz     = 16; N = bsz^2;
beta    = 4/5;
K_min   = 15; K_max = 60;
M_probe = 32; T1 = 41.27; T2 = 19.22;
M_S = 64; M_M = 96; M_C = 128;

% ── Sweep parameters ─────────────────────────────────────────────────────
Qb_range = 1:8;
Qb_ref   = 4;
M_sweep  = [32, 64, 128];
SR_sweep = M_sweep / N;

nD = size(DATASETS, 1);
nQ = length(Qb_range);
nM = length(M_sweep);

% ── Build sensing and DCT matrices once ──────────────────────────────────
fprintf('Building WHT and DCT matrices...\n');
H = hadamard(N);
seq = zeros(N, 1);
for i = 1:N
    seq(i) = sum(H(i,1:end-1) ~= H(i,2:end));
end
[~, ord]  = sort(seq);
Phi_full  = H(ord,:) / sqrt(N);
D         = dctmtx(bsz);
T_dct     = kron(D, D);

% ── Result storage ───────────────────────────────────────────────────────
PSNR = struct( ...
    'IPMC4',     zeros(nD,nQ), ...
    'Proposed7', zeros(nD,nQ)  ...
);
BPP = struct( ...
    'IPMC4',     zeros(nD,nQ), ...
    'Proposed7', zeros(nD,nQ)  ...
);
PSNR_IP4sweep  = zeros(nD, nM);
PSNR_SQsweep   = zeros(nD, nM);
PSNR_ADPpoint  = zeros(nD, 1);
SR_ADPpoint    = zeros(nD, 1);

mismatch_found = false;
valid = false(nD, 1);

% ── Main loop ────────────────────────────────────────────────────────────
for di = 1:nD
    seqname = DATASETS{di,2};
    fpath   = [IMG_DIR DATASETS{di,1}];

    if ~exist(fpath, 'file')
        fprintf('SKIPPING %s - file not found\n', seqname);
        continue;
    end
    valid(di) = true;

    raw = imread(fpath);
    if size(raw,3) == 3, raw = rgb2gray(raw); end
    img  = double(raw);
    rows = floor(size(img,1)/bsz) * bsz;
    cols = floor(size(img,2)/bsz) * bsz;
    img  = img(1:rows, 1:cols);
    numR = rows / bsz;
    numC = cols / bsz;

    fprintf('\n=== %s (%dx%d) ===\n', seqname, rows, cols);

    % RD sweep Qb=1..8
    for qi = 1:nQ
        Qb = Qb_range(qi);

        [r_i4, b_i4] = ipmc4_run(img, Phi_full, T_dct, numR, numC, bsz, N, Qb, beta);
        [r_p7, b_p7] = adpv2_run(img, Phi_full, T_dct, numR, numC, bsz, N, Qb, beta, ...
                                   K_min, K_max, M_probe, T1, T2, M_S, M_M, M_C);

        PSNR.IPMC4(di,qi)     = psnr_calc(img, r_i4);
        BPP.IPMC4(di,qi)      = b_i4 / (rows*cols);
        PSNR.Proposed7(di,qi) = psnr_calc(img, r_p7);
        BPP.Proposed7(di,qi)  = b_p7 / (rows*cols);

        fprintf('  Qb=%d  IPMC4=%.2f  ADPv2=%.2f dB\n', ...
            Qb, PSNR.IPMC4(di,qi), PSNR.Proposed7(di,qi));

        % Sanity check at Qb=4
        if Qb == Qb_ref && isfield(THESIS_REF, seqname)
            ref = THESIS_REF.(seqname);
            d_a = abs(PSNR.Proposed7(di,qi) - ref.adp);
            d_i = abs(PSNR.IPMC4(di,qi)     - ref.ip4);
            fprintf('  Sanity: ADPv2 diff=%.4f dB [%s]  IPMC4 diff=%.4f dB [%s]\n', ...
                d_a, status(d_a,TOL_DB), d_i, status(d_i,TOL_DB));
            if d_a > TOL_DB || d_i > TOL_DB
                mismatch_found = true;
                warning('%s: PSNR mismatch vs thesis Table 4.1', seqname);
            end
        end
    end

    % SR sweep at Qb=4
    for mi = 1:nM
        Mfix = M_sweep(mi);
        r_sq = nopred_fixed(img, Phi_full, T_dct, numR, numC, bsz, N, Qb_ref, Mfix);
        r_i4 = ipmc4_run(img, Phi_full, T_dct, numR, numC, bsz, N, Qb_ref, beta, Mfix);
        PSNR_SQsweep(di,mi)  = psnr_calc(img, r_sq);
        PSNR_IP4sweep(di,mi) = psnr_calc(img, r_i4);
    end

    % ADPv2 adaptive operating point
    [r_adp, ~, md] = adpv2_run(img, Phi_full, T_dct, numR, numC, bsz, N, ...
        Qb_ref, beta, K_min, K_max, M_probe, T1, T2, M_S, M_M, M_C);
    PSNR_ADPpoint(di) = psnr_calc(img, r_adp);
    nB = numR * numC;
    SR_ADPpoint(di) = (M_S*md(1) + M_M*md(2) + M_C*md(3)) / (nB*N);
    fprintf('  ADPv2 adaptive: avg SR=%.3f  PSNR=%.2f dB\n', ...
        SR_ADPpoint(di), PSNR_ADPpoint(di));
end

% ── Sanity check gate ────────────────────────────────────────────────────
if mismatch_found
    fprintf('\n*** MISMATCH DETECTED - do not use figures ***\n');
    return;
end
fprintf('\nAll sequences matched thesis Table 4.1 within %.2f dB\n', TOL_DB);

% ── Generate figures ─────────────────────────────────────────────────────
gain = PSNR.Proposed7 - PSNR.IPMC4;
plot_figures(PSNR, BPP, PSNR_IP4sweep, PSNR_SQsweep, ...
    PSNR_ADPpoint, SR_ADPpoint, gain, valid, DATASETS, ...
    Qb_range, SR_sweep, Qb_ref, OUT_DIR);

fprintf('\nDone. Figures saved to: %s\n', OUT_DIR);


% =========================================================================
%  Local helper: NoPred at fixed M (for SR sweep only)
% =========================================================================
function recon = nopred_fixed(img, Phi_full, T_dct, numR, numC, bsz, N, Qb, M)
rows  = numR*bsz; cols = numC*bsz;
recon = zeros(rows, cols);
Phi_m = Phi_full(1:M,:); A_m = Phi_m*T_dct';
K = max(15, min(60, M-1));
for r = 1:numR
    for c = 1:numC
        r0 = (r-1)*bsz+1; c0 = (c-1)*bsz+1;
        x  = img(r0:r0+bsz-1, c0:c0+bsz-1); x = x(:);
        y  = Phi_m*x;
        rng   = max(y) - min(y);
        Delta = max(rng/(2^Qb), 1e-6);
        y_hat = round(y/Delta)*Delta;
        th = omp_solve(A_m, y_hat, K);
        recon(r0:r0+bsz-1, c0:c0+bsz-1) = ...
            min(max(reshape(T_dct'*th, bsz, bsz), 0), 255);
    end
end
end

function s = status(diff, tol)
if diff > tol, s = 'MISMATCH'; else, s = 'OK'; end
end
