function qp = create_human_prediction_qp(params,model,humanVehicleIndex,humanState,previousHumanInput)
%CREATE_HUMAN_PREDICTION_QP Build the QP used to predict the manual driver's future input changes.
% Inputs:
%   params             : Project parameters and hard limits.
%   model              : Platoon model containing each individual vehicle model.
%   humanVehicleIndex  : Index of the manually controlled vehicle.
%   humanState         : Current human-vehicle state [position; velocity; acceleration].
%   previousHumanInput : Input applied to the human vehicle at sample k-1.
% Output:
%   qp : Quadratic program with N decision variables DeltaU_h.
% Objective:
%   J_h = sum(rDelta*DeltaU_h^2)
% Constraints:
%   vMin <= v_h <= vMax,  aMin <= a_h <= aMax over the prediction horizon.

% --- Build the single-vehicle lifted prediction model for the human-driven vehicle. ---
vehicleA = model.vehicleStateMatrix{humanVehicleIndex};
vehicleB = model.vehicleInputMatrix{humanVehicleIndex};
prediction = create_prediction_matrices(vehicleA,vehicleB,params.predictionHorizon);

% --- Encode one-step speed and acceleration bounds as G_h*x + g_h <= 0. ---
perStepMatrix = [0 -1 0;
                 0  1 0;
                 0  0 -1;
                 0  0  1];
perStepOffset = [params.minimumSpeed;
                 -params.maximumSpeed;
                  params.minimumAcceleration;
                 -params.maximumAcceleration];

% --- Repeat the one-step constraints over the full prediction horizon. ---
horizonConstraintMatrix = kron(speye(params.predictionHorizon),sparse(perStepMatrix));
horizonConstraintOffset = repmat(perStepOffset,params.predictionHorizon,1);

% --- Separate the known future trajectory from the effect of future input changes. ---
basePredictedState = prediction.stateFromCurrent*humanState + ...
    prediction.stateFromPreviousInput*previousHumanInput;
decisionEffectMatrix = prediction.stateFromInputChanges;

% --- Convert the human prediction problem to standard QP matrices. ---
qp.H = 2*params.inputChangeWeight*speye(params.predictionHorizon);
qp.f = zeros(params.predictionHorizon,1);
qp.A = horizonConstraintMatrix*decisionEffectMatrix;
qp.b = -horizonConstraintMatrix*basePredictedState-horizonConstraintOffset;
qp.basePredictedState = basePredictedState;
qp.decisionEffectMatrix = decisionEffectMatrix;
end
