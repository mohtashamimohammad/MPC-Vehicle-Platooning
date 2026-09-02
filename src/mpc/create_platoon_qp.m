function qp = create_platoon_qp(params,prediction,cost,currentState,previousInput, ...
    referenceStateStack,constraints,controlledVehicleIndices,humanInputChangeStack)
%CREATE_PLATOON_QP Build the constrained centralized MPC quadratic program.
% Inputs:
%   params                   : Project parameters and input-change weight.
%   prediction               : Lifted prediction matrices.
%   cost                     : Horizon cost matrices Omega and Psi.
%   currentState             : Current full platoon state X(k).
%   previousInput            : Previously applied input U(k-1).
%   referenceStateStack      : [X*(k+1); ...; X*(k+N)].
%   constraints              : Lifted hard state constraints.
%   controlledVehicleIndices : Vehicles whose DeltaU values are optimized by the MPC.
%   humanInputChangeStack    : Known/predicted manual-driver DeltaU values in the full NM vector.
% Output:
%   qp : H, f, A, b and matrices required to reconstruct the predicted state.
% Prediction form:
%   Xpred = Xbase + Bdecision*z
% QP form:
%   min 0.5*z'*H*z + f'*z,   subject to A*z <= b

% --- Determine the reduced decision-vector dimension for the currently controlled vehicles. ---
vehicleCount = params.vehicleCount;
horizonLength = prediction.horizonLength;
controlledVehicleIndices = controlledVehicleIndices(:).';
controlledVehicleCount = numel(controlledVehicleIndices);

% --- Map reduced MPC decisions into the full N*M input-change vector. ---
vehicleSelector = eye(vehicleCount);
vehicleSelector = vehicleSelector(:,controlledVehicleIndices);
fullDecisionMapping = kron(speye(horizonLength),sparse(vehicleSelector));
decisionEffectMatrix = prediction.stateFromInputChanges*fullDecisionMapping;

% --- Build the known part of the future state, including predicted manual-driver input changes. ---
basePredictedState = prediction.stateFromCurrent*currentState + ...
    prediction.stateFromPreviousInput*previousInput + ...
    prediction.stateFromInputChanges*humanInputChangeStack;
freeTrackingError = basePredictedState-referenceStateStack;

% --- Expand the tracking cost and input-change penalty into QP Hessian and gradient terms. ---
controlledInputWeight = params.inputChangeWeight*speye(horizonLength*controlledVehicleCount);
quadraticTerm = controlledInputWeight + ...
    decisionEffectMatrix.'*cost.stateWeightMatrix*decisionEffectMatrix;
linearTerm = decisionEffectMatrix.'*cost.stateWeightMatrix*freeTrackingError;

% --- Build the standard inequality-constrained QP and retain prediction data for reconstruction. ---
qp.H = sparse(2*quadraticTerm);
qp.H = 0.5*(qp.H+qp.H.');
qp.f = 2*linearTerm;
qp.A = constraints.horizonMatrix*decisionEffectMatrix;
qp.b = -constraints.horizonMatrix*basePredictedState-constraints.horizonOffset;
qp.basePredictedState = basePredictedState;
qp.decisionEffectMatrix = decisionEffectMatrix;
qp.controlledVehicleIndices = controlledVehicleIndices;
end
