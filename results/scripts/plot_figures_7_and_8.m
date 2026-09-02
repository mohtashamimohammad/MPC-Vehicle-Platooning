function plot_figures_7_and_8(results,params)
%PLOT_FIGURES_7_AND_8 Plot and save the LQR comparison figures.
% Inputs:
%   results : Output of simulate_lqr_comparison.
%   params  : Project parameters used for axis limits and constraint lines.
% Output files:
%   Figure 7: LQR inter-vehicle distances.
%   Figure 8: LQR vehicle velocities.

% --- Shared plotting styles. ---
lineStyles = {'-','--',':','-.','-'};

% --- Figure 7: LQR inter-vehicle distances. ---
figure7 = figure('Name','Figure 7','Color','w');
hold on;
for gapIndex = 1:params.vehicleCount-1
    plot(results.time,results.gaps(gapIndex,:),lineStyles{gapIndex},'LineWidth',1.2);
end
yline(params.minimumGap,'k-');
yline(params.maximumGap,'k-');
grid on;
xlim([0 params.lqrSimulationFinalTime]); ylim([-20 120]);
xlabel('Time [s]'); ylabel('Inter-vehicle distance [m]');
title('Figure 7 - LQR inter-vehicle distances');
legend({'Vehicle 1-2','Vehicle 2-3','Vehicle 3-4','Vehicle 4-5'},'Location','northeast');
savefig(figure7,fullfile('Results','Figure_7.fig'));
exportgraphics(figure7,fullfile('Results','Figure_7.png'),'Resolution',200);

% --- Figure 8: LQR vehicle velocities. ---
figure8 = figure('Name','Figure 8','Color','w');
hold on;
for vehicleIndex = 1:params.vehicleCount
    plot(results.time,results.velocities(vehicleIndex,:),lineStyles{vehicleIndex},'LineWidth',1.2);
end
yline(params.minimumSpeed,'k-');
yline(params.maximumSpeed,'k-');
grid on;
xlim([0 params.lqrSimulationFinalTime]); ylim([-10 60]);
xlabel('Time [s]'); ylabel('Velocity [m/s]');
title('Figure 8 - LQR vehicle velocities');
legend(arrayfun(@(index)sprintf('Vehicle %d',index),1:params.vehicleCount,'UniformOutput',false), ...
    'Location','southeast');
savefig(figure8,fullfile('Results','Figure_8.fig'));
exportgraphics(figure8,fullfile('Results','Figure_8.png'),'Resolution',200);
end
