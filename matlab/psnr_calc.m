function p = psnr_calc(orig, recon)
% psnr_calc  Compute PSNR between original and reconstructed image
%   orig  - original image (double)
%   recon - reconstructed image (double)
%   p     - PSNR in dB

mse = mean((orig(:) - recon(:)).^2);
p   = 10 * log10(255^2 / max(mse, 1e-12));
end
