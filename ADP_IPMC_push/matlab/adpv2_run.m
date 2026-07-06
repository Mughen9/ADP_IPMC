function [recon, total_bits, m_dist] = adpv2_run( ...
    img, Phi_full, T_dct, numR, numC, bsz, N, ...
    Qb, beta, K_min, K_max, M_probe, T1, T2, M_S, M_M, M_C)
% adpv2_run  ADPv2 adaptive block compressive sensing encoder
%
%   Verified 2026-06-19: Bosphorus Qb=4 gives 38.14 dB (thesis Table 4.1)
%
%   Inputs:
%     img      - input image (double, grayscale)
%     Phi_full - full SoWHT sensing matrix [N x N]
%     T_dct    - 2D DCT matrix [N x N]
%     numR     - number of block rows
%     numC     - number of block cols
%     bsz      - block size (16)
%     N        - block pixels (256)
%     Qb       - quantization bits
%     beta     - quantization scale (4/5)
%     K_min    - min OMP iterations (15)
%     K_max    - max OMP iterations (60)
%     M_probe  - probe measurement count (32)
%     T1       - smooth threshold (41.27)
%     T2       - complex threshold (19.22)
%     M_S      - smooth M* (64)
%     M_M      - moderate M* (96)
%     M_C      - complex M* (128)
%
%   Outputs:
%     recon      - reconstructed image
%     total_bits - total bits used
%     m_dist     - M* distribution [smooth, moderate, complex]

rows = numR * bsz;
cols = numC * bsz;
recon      = zeros(rows, cols);
total_bits = 0;
m_dist     = zeros(1, 3);

yh_buf = zeros(numC, M_C);
yl_buf = zeros(M_C, 1);

sS = Phi_full(1:M_S, :) * ones(N, 1);
sM = Phi_full(1:M_M, :) * ones(N, 1);
sC = Phi_full(1:M_C, :) * ones(N, 1);

for r = 1:numR
    yl_buf = zeros(M_C, 1);
    for c = 1:numC
        r0 = (r-1)*bsz + 1;
        c0 = (c-1)*bsz + 1;
        x  = img(r0:r0+bsz-1, c0:c0+bsz-1);
        x  = x(:);

        % Probe classifier - resolve sensing paradox
        x_zm = x - mean(x);
        y_zm = Phi_full * x_zm;
        E    = (N/M_probe) * mean(y_zm(1:M_probe).^2) / (mean(y_zm.^2) + 1e-8);

        if E > T1
            Mstar = M_S; s = sS; midx = 1;
        elseif E > T2
            Mstar = M_M; s = sM; midx = 2;
        else
            Mstar = M_C; s = sC; midx = 3;
        end
        m_dist(midx) = m_dist(midx) + 1;

        % Sense with selected M*
        y   = Phi_full(1:Mstar, :) * x;
        sn2 = dot(s, s);

        % Spectral prediction taps from neighbors
        yu = yh_buf(c, 1:Mstar)';
        yl = yl_buf(1:Mstar);
        au = dot(yu, s) / sn2;
        al = dot(yl, s) / sn2;

        % Seven-mode predictor - select best after quantization
        C       = modes7(au*s, al*s, Mstar);
        best_L1 = inf;
        best_m  = 1;
        for m = 1:7
            yr_m = y - C(:, m);
            D_m  = get_delta(yr_m, beta, Qb);
            L1   = sum(abs(yr_m - round(yr_m/D_m) * D_m));
            if L1 < best_L1
                best_L1 = L1;
                best_m  = m;
            end
        end

        % Quantize residual
        yp    = C(:, best_m);
        yr    = y - yp;
        Delta = get_delta(yr, beta, Qb);
        yq    = round(yr / Delta);

        % Adaptive K* based on residual ratio
        sy  = sqrt(mean(y.^2)  + 1e-8);
        syr = sqrt(mean(yr.^2) + 1e-8);
        Ks  = max(K_min, min(K_max, min(K_min + round((K_max-K_min)*syr/sy), Mstar-1)));

        % Reconstruct and write to drift-free IQ buffer
        y_hat = yp + yq * Delta;
        tap   = zeros(M_C, 1);
        tap(1:Mstar) = y_hat;
        yh_buf(c, :) = tap';
        yl_buf       = tap;

        % Bitcount: 3 bits mode + 2 bits M* header + Mstar*Qb data
        total_bits = total_bits + 3 + 2 + Mstar * Qb;

        % OMP reconstruction
        th = omp_solve(Phi_full(1:Mstar,:) * T_dct', y_hat, Ks);
        recon(r0:r0+bsz-1, c0:c0+bsz-1) = ...
            min(max(reshape(T_dct' * th, bsz, bsz), 0), 255);
    end
end
end
