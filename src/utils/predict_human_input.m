function prediction = predict_human_input(params,model,humanVehicleIndex,humanState,previousHumanInput)
%PREDICT_HUMAN_INPUT Predict the manual driver's input changes over the MPC horizon.
% Inputs:
%   params             : Project parameters.
%   model              : Platoon model.
%   humanVehicleIndex  : Index of the manually controlled vehicle.
%   humanState         : Current human-vehicle state [p;v;a].
%   previousHumanInput : Previously applied human-vehicle input.
% Output:
%   prediction.inputChanges   : Predicted DeltaU_h sequence over N samples.
%   prediction.predictedState : Corresponding predicted human-vehicle state stack.
% If DeltaU_h=0 is already feasible, it is the minimum-change solution and no QP solve is needed.

% --- Build the horizon QP for the manual vehicle. ---
humanQP = create_human_prediction_qp( ...
    params,model,humanVehicleIndex,humanState,previousHumanInput);
zeroInputChange = zeros(params.predictionHorizon,1);

% --- Use zero input change when it is already feasible; otherwise solve the human prediction QP. ---
if max(humanQP.A*zeroInputChange-humanQP.b) <= 0
    optimalInputChange = zeroInputChange;
    predictedHumanState = humanQP.basePredictedState;
else
    solution = solve_mpc_qp(humanQP);
    optimalInputChange = solution.decision;
    predictedHumanState = solution.predictedState;
end

% --- Return the predicted input-change sequence and state trajectory. ---
prediction.inputChanges = optimalInputChange;
prediction.predictedState = predictedHumanState;
end
