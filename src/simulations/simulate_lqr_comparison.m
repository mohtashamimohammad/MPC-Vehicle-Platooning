function results = simulate_lqr_comparison(params)
%SIMULATE_LQR_COMPARISON Simulate the unconstrained LQR comparison case.
% Input:
%   params : Project and LQR parameters.
% Output:
%   results : Time histories of positions, velocities, gaps, and applied LQR control.
% Control law:
%   u_raw = -K*(x-x_ref)
% Only acceleration is saturated to [aMin,aMax]; gap and speed limits are not enforced by the LQR.

% --- Build the LQR controller and initialize simulation dimensions. ---
controller = create_lqr_controller(params);
vehicleCount = params.vehicleCount;
sampleTime = params.sampleTime;
numberOfSteps = round(params.lqrSimulationFinalTime/sampleTime);
constantGap = params.lqrDesiredGap;

% --- Initialize all comparison-model positions and velocities at zero. ---
currentState = [zeros(vehicleCount,1);zeros(vehicleCount,1)];
stateHistory = zeros(2*vehicleCount,numberOfSteps+1);
appliedControlHistory = zeros(vehicleCount,numberOfSteps);
stateHistory(:,1) = currentState;

% --- Time-stepping loop for the LQR comparison. ---
for simulationStep = 1:numberOfSteps
    sampleIndex = simulationStep-1;
    currentTime = sampleIndex*sampleTime;

    % Build the moving reference with constant inter-vehicle spacing.
    referencePosition = params.lqrLeadInitialPosition + params.desiredSpeed*currentTime - ...
        (0:vehicleCount-1).'*constantGap;
    referenceVelocity = params.desiredSpeed*ones(vehicleCount,1);
    referenceState = [referencePosition;referenceVelocity];

    % Apply LQR feedback and saturate only the acceleration command.
    rawControl = -controller.gain*(currentState-referenceState);
    appliedControl = min(max(rawControl,params.minimumAcceleration),params.maximumAcceleration);

    % Propagate the double-integrator model exactly over one sample with constant acceleration.
    positions = currentState(1:vehicleCount);
    velocities = currentState(vehicleCount+1:2*vehicleCount);
    nextPositions = positions + sampleTime*velocities + 0.5*sampleTime^2*appliedControl;
    nextVelocities = velocities + sampleTime*appliedControl;
    currentState = [nextPositions;nextVelocities];

    stateHistory(:,simulationStep+1) = currentState;
    appliedControlHistory(:,simulationStep) = appliedControl;
end

% --- Extract time, positions, velocities, and physical gaps for plotting. ---
time = (0:numberOfSteps)*sampleTime;
positions = stateHistory(1:vehicleCount,:);
velocities = stateHistory(vehicleCount+1:2*vehicleCount,:);
gaps = positions(1:vehicleCount-1,:)-positions(2:vehicleCount,:);

results.time = time;
results.positions = positions;
results.velocities = velocities;
results.gaps = gaps;
results.appliedControl = appliedControlHistory;
end
