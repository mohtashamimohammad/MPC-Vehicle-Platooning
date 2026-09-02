function results = run_project()
%RUN_PROJECT Execute the complete cleaned project.
% Output:
%   results.mainScenario    : Main centralized MPC plus manual-driver scenario.
%   results.stringStability : String-stability simulation and attenuation metrics.
%   results.lqrComparison   : LQR comparison simulation.
% The function also creates Figures 3 through 8 in the Results folder.

% --- Load parameters and create the output directory. ---
params = platoon_parameters();
if ~exist('results','dir')
    mkdir('results');
end

% --- Main MPC/manual-driver scenario and Figures 3-5. ---
mainScenario = simulate_main_scenario(params);
plot_figures_3_to_5(mainScenario,params);

% --- String-stability experiment and Figure 6. ---
stringStability = simulate_string_stability(params);
plot_figure_6(stringStability,params);

% --- LQR comparison and Figures 7-8. ---
lqrComparison = simulate_lqr_comparison(params);
plot_figures_7_and_8(lqrComparison,params);

% --- Return all three simulation result structures to the caller. ---
results.mainScenario = mainScenario;
results.stringStability = stringStability;
results.lqrComparison = lqrComparison;
end
