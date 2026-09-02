function results = simulate_main_scenario(params)
%SIMULATE_MAIN_SCENARIO Simulate the complete 0-400 s centralized MPC scenario.
% Input:
%   params : Project parameters.
% Output:
%   results : State, input, reference, spacing, and manual-mode time histories.
% Main events:
%   100 s     : vehicle 3 leaves centralized MPC and brakes manually.
%   150 s     : the manual-driver target speed changes to 11 m/s.
%   250 s     : vehicle 3 rejoins centralized MPC.
%   320-355 s : headways transition to their final values.
% Receding-horizon operation:
%   At each sample, build Xref, solve the QP, apply only the first DeltaU block,
%   update U(k)=U(k-1)+DeltaU(k), then propagate X(k+1)=A*X(k)+B*U(k).

% --- Build the plant, lifted prediction model, and lifted hard constraints. ---
model = create_platoon_model(params);
prediction = create_prediction_matrices( ...
    model.stateMatrix,model.inputMatrix,params.predictionHorizon);
constraints = create_state_constraints(params,params.predictionHorizon);

% --- Precompute horizon costs before and after the headway transition. ---
costBeforeHeadwayChange = create_horizon_cost(params,model,0,params.predictionHorizon);
costAfterHeadwayChange = create_horizon_cost( ...
    params,model,params.headwayTransitionEndSample,params.predictionHorizon);

% --- Read dimensions and scenario indices. ---
vehicleCount = params.vehicleCount;
stateCount = model.stateCount;
humanVehicleIndex = params.humanVehicleIndex;
numberOfSteps = round(params.mainSimulationFinalTime/params.sampleTime);

% --- Initialize the platoon at rest with the desired physical gaps. ---
initialHeadway = get_headway(params,0);
initialSpacing = calculate_desired_spacing(params,0,initialHeadway);
initialPosition = -[0;cumsum(initialSpacing(2:end))];
initialState = [initialPosition;zeros(vehicleCount,1);zeros(vehicleCount,1)];
previousInput = zeros(vehicleCount,1);
referenceState = initialize_reference(params,initialState,0,0);

% --- Preallocate the simulation histories. ---
stateHistory = zeros(stateCount,numberOfSteps+1);
inputHistory = zeros(vehicleCount,numberOfSteps);
inputChangeHistory = zeros(vehicleCount,numberOfSteps);
referenceHistory = zeros(stateCount,numberOfSteps+1);
desiredSpacingHistory = zeros(vehicleCount,numberOfSteps+1);
humanCommandHistory = nan(1,numberOfSteps);
manualMode = false(1,numberOfSteps+1);
stateHistory(:,1) = initialState;
wasManual = false;

% --- Receding-horizon closed-loop simulation. ---
for simulationStep = 1:numberOfSteps
    sampleIndex = simulationStep-1;
    currentTime = sampleIndex*params.sampleTime;
    currentState = stateHistory(:,simulationStep);

    % Determine whether the human vehicle is currently outside centralized MPC.
    isManual = currentTime >= params.emergencyBrakeTime && currentTime < params.rejoinTime;
    manualMode(simulationStep) = isManual;

    % Re-anchor the reference to the manual vehicle, then restore normal anchoring at rejoin.
    if isManual
        referenceState = initialize_reference( ...
            params,currentState,sampleIndex,humanVehicleIndex);
    elseif wasManual
        referenceState = initialize_reference(params,currentState,sampleIndex,0);
    end

    % Store the current reference and build the N-step future reference stack.
    currentReference = get_platoon_reference(params,referenceState,sampleIndex);
    referenceHistory(:,simulationStep) = currentReference.stateVector;
    desiredSpacingHistory(:,simulationStep) = currentReference.desiredSpacing;
    referenceHorizon = create_reference_horizon( ...
        params,referenceState,sampleIndex,params.predictionHorizon);

    % Predict the manual vehicle input changes and remove that vehicle from the central decision vector.
    humanInputChangeStack = zeros(params.predictionHorizon*vehicleCount,1);
    if isManual
        humanState = [currentState(humanVehicleIndex); ...
            currentState(vehicleCount+humanVehicleIndex); ...
            currentState(2*vehicleCount+humanVehicleIndex)];
        humanPrediction = predict_human_input( ...
            params,model,humanVehicleIndex,humanState,previousInput(humanVehicleIndex));

        for predictionStep = 1:params.predictionHorizon
            stackIndex = (predictionStep-1)*vehicleCount+humanVehicleIndex;
            humanInputChangeStack(stackIndex) = humanPrediction.inputChanges(predictionStep);
        end
        controlledVehicleIndices = setdiff(1:vehicleCount,humanVehicleIndex,'stable');
    else
        controlledVehicleIndices = 1:vehicleCount;
    end

    % Select the appropriate cost matrix for the current headway schedule.
    if sampleIndex+params.predictionHorizon <= params.headwayChangeSample
        currentCost = costBeforeHeadwayChange;
    elseif sampleIndex >= params.headwayTransitionEndSample
        currentCost = costAfterHeadwayChange;
    else
        currentCost = create_horizon_cost( ...
            params,model,sampleIndex,params.predictionHorizon);
    end

    % Build and solve the centralized constrained QP.
    platoonQP = create_platoon_qp( ...
        params,prediction,currentCost,currentState,previousInput, ...
        referenceHorizon.stateStack,constraints,controlledVehicleIndices, ...
        humanInputChangeStack);
    centralSolution = solve_mpc_qp(platoonQP);

    % Apply only the first DeltaU block from the optimal horizon decision.
    centralInputChange = zeros(vehicleCount,1);
    centralInputChange(controlledVehicleIndices) = ...
        centralSolution.decision(1:numel(controlledVehicleIndices));
    appliedInputChange = centralInputChange;

    if isManual
        humanState = [currentState(humanVehicleIndex); ...
            currentState(vehicleCount+humanVehicleIndex); ...
            currentState(2*vehicleCount+humanVehicleIndex)];

        % Replace the human-vehicle MPC component with the actual manual command.
        humanCommand = human_driver_command( ...
            params,model,humanVehicleIndex,humanState,sampleIndex);
        appliedInputChange(humanVehicleIndex) = ...
            humanCommand.input-previousInput(humanVehicleIndex);
        humanCommandHistory(simulationStep) = humanCommand.input;
    end

    % Update the absolute input and propagate the centralized vehicle model by one sample.
    currentInput = previousInput+appliedInputChange;
    nextState = model.stateMatrix*currentState+model.inputMatrix*currentInput;

    inputChangeHistory(:,simulationStep) = appliedInputChange;
    inputHistory(:,simulationStep) = currentInput;
    stateHistory(:,simulationStep+1) = nextState;
    previousInput = currentInput;
    wasManual = isManual;
end

% --- Complete the final reference sample so all state histories have equal length. ---
finalReference = get_platoon_reference(params,referenceState,numberOfSteps);
referenceHistory(:,end) = finalReference.stateVector;
desiredSpacingHistory(:,end) = finalReference.desiredSpacing;

% --- Extract physical signals and gaps from the centralized state history. ---
positions = stateHistory(1:vehicleCount,:);
velocities = stateHistory(vehicleCount+1:2*vehicleCount,:);
accelerations = stateHistory(2*vehicleCount+1:3*vehicleCount,:);
gaps = positions(1:vehicleCount-1,:)-positions(2:vehicleCount,:);
physicalDesiredGaps = desiredSpacingHistory(2:vehicleCount,:);
time = (0:numberOfSteps)*params.sampleTime;

% --- Package the simulation histories for plotting and analysis. ---
results.params = params;
results.time = time;
results.stateHistory = stateHistory;
results.inputHistory = inputHistory;
results.inputChangeHistory = inputChangeHistory;
results.referenceHistory = referenceHistory;
results.desiredSpacingHistory = desiredSpacingHistory;
results.positions = positions;
results.velocities = velocities;
results.accelerations = accelerations;
results.gaps = gaps;
results.physicalDesiredGaps = physicalDesiredGaps;
results.humanCommandHistory = humanCommandHistory;
results.manualMode = manualMode;
end
