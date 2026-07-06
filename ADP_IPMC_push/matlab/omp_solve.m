function th = omp_solve(A, y, K)
% omp_solve  Orthogonal Matching Pursuit
%   A  - sensing matrix [M x Nd]
%   y  - measurement vector [M x 1]
%   K  - max iterations
%   th - sparse coefficient vector [Nd x 1]

[~, Nd] = size(A);
r   = y;
idx = [];
th  = zeros(Nd, 1);
K   = min(K, size(A,1) - 1);

for k = 1:K
    [~, i] = max(abs(A' * r));
    if any(idx == i), break; end
    idx  = [idx, i];
    coef = A(:, idx) \ y;
    r    = y - A(:, idx) * coef;
    if norm(r) < 1e-8, break; end
end

if ~isempty(idx)
    th(idx) = A(:, idx) \ y;
end
end
