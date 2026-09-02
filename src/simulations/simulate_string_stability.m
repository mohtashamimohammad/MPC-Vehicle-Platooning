function results = simulate_string_stability(params)
%SIMULATE_STRING_STABILITY Evaluate disturbance propagation through the platoon.
% Input:
%   params : Project and string-stability parameters.
% Output:
%   results : State histories, gap errors, disturbance history, and attenuation metrics.
% Metrics:
%   Peak_i = max(abs(e_gap,i))
%   RMS_i  = sqrt(mean(e_gap,i^2))
%   attenuation_i = metric_i/metric_1
% Ratios below 1 indicate attenuation relative to the first inter-vehicle gap.

% --- Build the same centralized MPC model used by the main scenario. ---
model = create_platoon_model(params);
prediction = create_prediction_matrices( ...
    model.stateMatrix,model.inputMatrix,params.predictionHorizon);
constraints = create_state_constraints(params,params.predictionHorizon);
cost = create_horizon_cost(params,model,0,params.predictionHorizon);

% --- Set simulation dimensions and duration. ---
vehicleCount = params.vehicleCount;
numberOfSteps = round(params.stringSimulationFinalTime/params.sampleTime);

% --- Initialize the platoon at the nominal desired gaps. ---
initialHeadway = get_headway(params,0);
initialSpacing = calculate_desired_spacing(params,0,initialHeadway);
initialPosition = -[0;cumsum(initialSpacing(2:end))];
initialState = [initialPosition;zeros(vehicleCount,1);zeros(vehicleCount,1)];
previousInput = zeros(vehicleCount,1);
referenceState = initialize_reference(params,initialState,0,0);

% --- Preallocate simulation histories and define the full MPC-controlled vehicle set. ---
stateHistory = zeros(model.stateCount,numberOfSteps+1);
desiredSpacingHistory = zeros(vehicleCount,numberOfSteps+1);
disturbanceRate = zeros(1,numberOfSteps);
stateHistory(:,1) = initialState;

controlledVehicleIndices = 1:vehicleCount;
noHumanInputChange = zeros(params.predictionHorizon*vehicleCount,1);

% --- Closed-loop MPC simulation with an additive lead-vehicle disturbance. ---
for simulationStep = 1:numberOfSteps
    sampleIndex = simulationStep-1;
    currentState = stateHistory(:,simulationStep);

    % Build the current and future reference trajectory.
    currentReference = get_platoon_reference(params,referenceState,sampleIndex);
    desiredSpacingHistory(:,simulationStep) = currentReference.desiredSpacing;
    referenceHorizon = create_reference_horizon( ...
        params,referenceState,sampleIndex,params.predictionHorizon);

    % Solve the centralized MPC QP with no manual driver.
    platoonQP = create_platoon_qp( ...
        params,prediction,cost,currentState,previousInput, ...
        referenceHorizon.stateStack,constraints,controlledVehicleIndices, ...
        noHumanInputChange);
    solution = solve_mpc_qp(platoonQP);

    % Apply only the first optimal DeltaU block and propagate the nominal plant.
    appliedInputChange = solution.decision(1:vehicleCount);
    currentInput = previousInput+appliedInputChange;
    nominalNextState = model.stateMatrix*currentState+model.inputMatrix*currentInput;

    % Add the configured position disturbance to the next state.
    disturbance = get_string_disturbance(params,sampleIndex);
    nextState = nominalNextState+disturbance.stateIncrement;

    stateHistory(:,simulationStep+1) = nextState;
    disturbanceRate(simulationStep) = disturbance.positionRate;
    previousInput = currentInput;
end

% --- Complete the final desired-spacing sample. ---
finalReference = get_platoon_reference(params,referenceState,numberOfSteps);
desiredSpacingHistory(:,end) = finalReference.desiredSpacing;

% --- Extract physical gaps and compute gap-tracking error. ---
positions = stateHistory(1:vehicleCount,:);
velocities = stateHistory(vehicleCount+1:2*vehicleCount,:);
accelerations = stateHistory(2*vehicleCount+1:3*vehicleCount,:);
gaps = positions(1:vehicleCount-1,:)-positions(2:vehicleCount,:);
physicalDesiredGaps = desiredSpacingHistory(2:vehicleCount,:);
gapError = gaps-physicalDesiredGaps;
time = (0:numberOfSteps)*params.sampleTime;

% --- Evaluate peak and RMS attenuation from disturbance start through the recovery window. ---
analysisMask = time >= params.disturbanceStartTime & ...
    time <= params.stringAnalysisEndTime;
peakGapDeviation = max(abs(gapError(:,analysisMask)),[],2);
rmsGapDeviation = sqrt(mean(gapError(:,analysisMask).^2,2));
peakAttenuationRatio = peakGapDeviation/max(peakGapDeviation(1),eps);
rmsAttenuationRatio = rmsGapDeviation/max(rmsGapDeviation(1),eps);

% --- Package the simulation histories and string-stability metrics. ---
results.params = params;
results.time = time;
results.positions = positions;
results.velocities = velocities;
results.accelerations = accelerations;
results.gaps = gaps;
results.gapError = gapError;
results.disturbanceRate = disturbanceRate;
results.peakGapDeviation = peakGapDeviation;
results.rmsGapDeviation = rmsGapDeviation;
results.peakAttenuationRatio = peakAttenuationRatio;
results.rmsAttenuationRatio = rmsAttenuationRatio;
end
