function plot_figure_6(results,params)
%PLOT_FIGURE_6 Plot and save Figure 6.
% Inputs:
%   results : Output of simulate_string_stability.
%   params  : Project parameters used for axis limits and constraint lines.
% Output files:
%   Results/Figure_6.fig
%   Results/Figure_6.png

% --- Figure 6: inter-vehicle distances during the string-stability experiment. ---
figure6 = figure('Name','Figure 6','Color','w');
hold on;
lineStyles = {'-','--','-.',':'};
for gapIndex = 1:params.vehicleCount-1
    plot(results.time,results.gaps(gapIndex,:),lineStyles{gapIndex},'LineWidth',1.2);
end
yline(params.minimumGap,'k-');
yline(params.maximumGap,'k-');
grid on; box on;
xlim([params.stringFigureStartTime params.stringSimulationFinalTime]);
ylim([0 70]);
xlabel('Time [s]'); ylabel('Inter-vehicle distance [m]');
title('Figure 6 - String stability');
legend({'Vehicle 1-2','Vehicle 2-3','Vehicle 3-4','Vehicle 4-5'},'Location','northeast');

% --- Save both editable MATLAB and PNG versions. ---
savefig(figure6,fullfile('Results','Figure_6.fig'));
exportgraphics(figure6,fullfile('Results','Figure_6.png'),'Resolution',200);
end
