function plot_figures_3_to_5(results,params)
%PLOT_FIGURES_3_TO_5 Plot and save Figures 3, 4, and 5.
% Inputs:
%   results : Output of simulate_main_scenario.
%   params  : Project parameters used for axis limits and constraint lines.
% Output files:
%   Figure 3: inter-vehicle distances.
%   Figure 4: vehicle velocities.
%   Figure 5: vehicle accelerations.

% --- Shared plotting settings. ---
vehicleCount = params.vehicleCount;
time = results.time;
lineStyles = {'-','--',':','-.','-'};

% --- Figure 3: inter-vehicle distances. ---
figure3 = figure('Name','Figure 3','Color','w');
hold on;
for gapIndex = 1:vehicleCount-1
    plot(time,results.gaps(gapIndex,:),lineStyles{gapIndex},'LineWidth',1.2);
end
yline(params.minimumGap,'k-');
yline(params.maximumGap,'k-');
grid on; box on;
xlim([0 params.mainSimulationFinalTime]); ylim([0 70]);
xlabel('Time [s]'); ylabel('Inter-vehicle distance [m]');
title('Figure 3 - Inter-vehicle distances');
legend({'Vehicle 1-2','Vehicle 2-3','Vehicle 3-4','Vehicle 4-5'},'Location','northwest');
savefig(figure3,fullfile('Results','Figure_3.fig'));
exportgraphics(figure3,fullfile('Results','Figure_3.png'),'Resolution',200);

% --- Figure 4: vehicle velocities. ---
figure4 = figure('Name','Figure 4','Color','w');
hold on;
for vehicleIndex = 1:vehicleCount
    plot(time,results.velocities(vehicleIndex,:),lineStyles{vehicleIndex},'LineWidth',1.1);
end
grid on; box on;
xlim([0 params.mainSimulationFinalTime]); ylim([-5 30]);
xlabel('Time [s]'); ylabel('Velocity [m/s]');
title('Figure 4 - Vehicle velocities');
legend(arrayfun(@(index)sprintf('Vehicle %d',index),1:vehicleCount,'UniformOutput',false), ...
    'Location','best');
savefig(figure4,fullfile('Results','Figure_4.fig'));
exportgraphics(figure4,fullfile('Results','Figure_4.png'),'Resolution',200);

% --- Figure 5: vehicle accelerations. ---
figure5 = figure('Name','Figure 5','Color','w');
hold on;
for vehicleIndex = 1:vehicleCount
    plot(time,results.accelerations(vehicleIndex,:),lineStyles{vehicleIndex},'LineWidth',1.1);
end
yline(params.minimumAcceleration,'k-');
yline(params.maximumAcceleration,'k-');
grid on; box on;
xlim([0 params.mainSimulationFinalTime]); ylim([-8 4]);
xlabel('Time [s]'); ylabel('Acceleration [m/s^2]');
title('Figure 5 - Vehicle accelerations');
legend(arrayfun(@(index)sprintf('Vehicle %d',index),1:vehicleCount,'UniformOutput',false), ...
    'Location','best');
savefig(figure5,fullfile('Results','Figure_5.fig'));
exportgraphics(figure5,fullfile('Results','Figure_5.png'),'Resolution',200);
end
