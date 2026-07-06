function Delta = get_delta(yr, beta, Qb)
% get_delta  Compute adaptive quantization step size
%   yr    - residual vector
%   beta  - scaling factor (4/5 in ADPv2)
%   Qb    - quantization bits
%   Delta - quantization step size

sigma = sqrt(mean(yr.^2) + 1e-8);
Delta = beta * sigma / (2^((Qb - 4) / 2));
Delta = max(min(Delta, 2^7), 2^-4);
end
