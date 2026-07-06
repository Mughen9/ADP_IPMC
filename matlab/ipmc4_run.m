function [recon, total_bits] = ipmc4_run( ...
    img, Phi_full, T_dct, numR, numC, bsz, N, Qb, beta, M)
% ipmc4_run  IPMC-4 baseline encoder (Peetakul et al., TCSVT 2022)
%
%   Used for comparison only - not part of ADPv2 contribution.
%   Fixed M measurements, 4-mode predictor, no adaptive-M or adaptive-K.
%
%   If M is not provided, defaults to 64 (published configuration).

if nargin < 10
    M = 64;
end
K = max(15, min(60, M-1));

rows = numR * bsz;
cols = numC * bsz;
recon      = zeros(rows, cols);
total_bits = 0;

yh_buf = zeros(numC, M);
yl_buf = zeros(M, 1);
Phi_m  = Phi_full(1:M, :);
A_m    = Phi_m * T_dct';
s      = Phi_m * ones(N, 1);
sn2    = dot(s, s);

for r = 1:numR
    yl_buf = zeros(M, 1);
    for c = 1:numC
        r0 = (r-1)*bsz + 1;
        c0 = (c-1)*bsz + 1;
        x  = img(r0:r0+bsz-1, c0:c0+bsz-1);
        x  = x(:);
        y  = Phi_m * x;

        yu = yh_buf(c, :)';
        yl = yl_buf;
        au = dot(yu, s) / sn2;
        al = dot(yl, s) / sn2;

        yu_p  = au * s;
        yl_p  = al * s;
        y_avg = 0.5 * (yu_p + yl_p);
        C4    = [yu_p, yl_p, y_avg, zeros(M,1)];

        [~, bm] = min(sum(abs(y - C4), 1));
        yp    = C4(:, bm);
        yr    = y - yp;
        Delta = get_delta(yr, beta, Qb);
        yq    = round(yr / Delta);

        yh_buf(c, :) = y';
        yl_buf       = y;
        y_hat        = yp + yq * Delta;

        total_bits = total_bits + 2 + M * Qb;

        th = omp_solve(A_m, y_hat, K);
        recon(r0:r0+bsz-1, c0:c0+bsz-1) = ...
            min(max(reshape(T_dct' * th, bsz, bsz), 0), 255);
    end
end
end
