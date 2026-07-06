function C = modes7(yu_p, yl_p, M)
% modes7  Seven-mode spectral predictor candidate matrix
%   yu_p - upper neighbor predicted measurement [M x 1]
%   yl_p - left neighbor predicted measurement  [M x 1]
%   M    - number of measurements
%   C    - candidate matrix [M x 7]
%
%   Mode 0: Upper
%   Mode 1: Left
%   Mode 2: Average(upper, left)
%   Mode 3: Weighted blend (content-adaptive)
%   Mode 4: SpecCut - low-sequency quarter only
%   Mode 5: SpecExp - decaying exponential envelope
%   Mode 6: Zero (boundary fallback)

y_avg = 0.5 * (yu_p + yl_p);

aw    = 1 ./ (1 + exp(-abs(yu_p - yl_p) / 20));
y_wt  = aw .* yu_p + (1 - aw) .* yl_p;

y_cut = zeros(M, 1);
y_cut(1:max(1, floor(M/4))) = y_avg(1:max(1, floor(M/4)));

k     = (0:M-1)';
y_exp = exp(-k * log(max(M, 2)) / (M/4)) .* y_avg;

C = [yu_p, yl_p, y_avg, y_wt, y_cut, y_exp, zeros(M,1)];
end
