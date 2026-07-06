function plot_figures(PSNR, BPP, PSNR_IP4sweep, PSNR_SQsweep, ...
    PSNR_ADPpoint, SR_ADPpoint, gain, valid, DATASETS, ...
    Qb_range, SR_sweep, Qb_ref, OUT_DIR)
% plot_figures  Generate all RD curve and gain figures
%
%   Called by adpv2_main after all sequences are processed.
%   Saves Fig1, Fig2, Fig3, Fig5 to OUT_DIR.

nD = size(DATASETS, 1);

C = struct( ...
    'NoPred',   [0.47 0.67 0.19], ...
    'IPMC4',    [0.85 0.33 0.10], ...
    'Proposed7',[0.00 0.45 0.74]  ...
);

% Figure 1: RD curve per sequence
for di = 1:nD
    if ~valid(di), continue; end
    sn  = DATASETS{di, 2};
    fig = figure('Position',[100 100 720 520]);
    hold on; grid on; box on;
    plot(BPP.IPMC4(di,:),     PSNR.IPMC4(di,:),     '-^', 'Color', C.IPMC4,     'LineWidth',1.8, 'MarkerSize',7);
    plot(BPP.Proposed7(di,:), PSNR.Proposed7(di,:), '-o', 'Color', C.Proposed7, 'LineWidth',2.4, 'MarkerSize',7);
    xlabel('BPP'); ylabel('PSNR (dB)');
    legend({'IPMC-4','ADPv2'}, 'Location','southeast');
    title(sprintf('RD Curve - %s (Qb=1..8)', sn));
    exportgraphics(fig, [OUT_DIR sprintf('Fig1_RD_%s.png', sn)], 'Resolution',200);
    close(fig);
    fprintf('Saved Fig1_RD_%s.png\n', sn);
end

% Figure 2: Averaged RD curve
avgBPP_I4 = mean(BPP.IPMC4(valid,:),     1);
avgBPP_P7 = mean(BPP.Proposed7(valid,:), 1);
avgPSNR_I4 = mean(PSNR.IPMC4(valid,:),     1);
avgPSNR_P7 = mean(PSNR.Proposed7(valid,:), 1);

fig = figure('Position',[100 100 720 520]);
hold on; grid on; box on;
plot(avgBPP_I4, avgPSNR_I4, '-^', 'Color', C.IPMC4,     'LineWidth',1.8, 'MarkerSize',7);
plot(avgBPP_P7, avgPSNR_P7, '-o', 'Color', C.Proposed7, 'LineWidth',2.4, 'MarkerSize',7);
xlabel('BPP'); ylabel('PSNR (dB)');
legend({'IPMC-4','ADPv2'}, 'Location','southeast');
title('RD Curve - Average of 4 Sequences');
exportgraphics(fig, [OUT_DIR 'Fig2_RD_Average.png'], 'Resolution',200);
close(fig);
fprintf('Saved Fig2_RD_Average.png\n');

% Figure 3: PSNR vs Sampling Rate
avg_IP4      = mean(PSNR_IP4sweep(valid,:),  1);
avg_SQ       = mean(PSNR_SQsweep(valid,:),   1);
avg_SR_ADP   = mean(SR_ADPpoint(valid));
avg_PSNR_ADP = mean(PSNR_ADPpoint(valid));

fig = figure('Position',[100 100 720 520]);
hold on; grid on; box on;
plot(SR_sweep, avg_SQ,  '-x', 'Color', C.NoPred,    'LineWidth',1.8, 'MarkerSize',8);
plot(SR_sweep, avg_IP4, '-^', 'Color', C.IPMC4,     'LineWidth',1.8, 'MarkerSize',8);
plot(avg_SR_ADP, avg_PSNR_ADP, 'p', 'Color', C.Proposed7, ...
    'MarkerSize',16, 'MarkerFaceColor', C.Proposed7);
xlabel('Sampling Rate (M/N)'); ylabel('PSNR (dB)');
legend({'NoPred (fixed M)','IPMC-4 (fixed M)', ...
    sprintf('ADPv2 (adaptive, avg SR=%.3f)', avg_SR_ADP)}, 'Location','southeast');
title(sprintf('PSNR vs Sampling Rate - Qb=%d', Qb_ref));
exportgraphics(fig, [OUT_DIR 'Fig3_PSNRvsSR.png'], 'Resolution',200);
close(fig);
fprintf('Saved Fig3_PSNRvsSR.png\n');

% Figure 5: ADPv2 gain over IPMC-4 vs Qb
avgGain  = mean(gain(valid,:), 1);
seqC     = lines(nD);
legE     = {};

fig = figure('Position',[100 100 720 520]);
hold on; grid on; box on;
for di = 1:nD
    if ~valid(di), continue; end
    plot(Qb_range, gain(di,:), '-o', 'Color', seqC(di,:), 'LineWidth',1.5, 'MarkerSize',6);
    legE{end+1} = DATASETS{di,2};
end
plot(Qb_range, avgGain, '-k', 'LineWidth',2.8);
legE{end+1} = 'Average';
yline(0, '--', 'HandleVisibility','off');
xlabel('Qb (bits)'); ylabel('\Delta PSNR (dB)  ADPv2 - IPMC-4');
legend(legE, 'Location','northeast');
title('ADPv2 Gain over IPMC-4 vs Qb');
exportgraphics(fig, [OUT_DIR 'Fig5_GainVsQb.png'], 'Resolution',200);
close(fig);
fprintf('Saved Fig5_GainVsQb.png\n');
end
