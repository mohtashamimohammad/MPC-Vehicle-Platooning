function cost = create_horizon_cost(params,model,currentSample,horizonLength)
%CREATE_HORIZON_COST Assemble the state and input-change weights over the MPC horizon.
% Inputs:
%   params        : Project parameters and MPC weights.
%   model         : Centralized platoon model containing A and B.
%   currentSample : Current discrete-time sample k.
%   horizonLength : Prediction horizon N.
% Output:
%   cost.stateWeightMatrix       : Omega, the block-diagonal state-error weight.
%   cost.inputChangeWeightMatrix : Psi, the block-diagonal DeltaU weight.
% Cost used by the MPC:
%   J = (Xpred-Xref)'*Omega*(Xpred-Xref) + DeltaU'*Psi*DeltaU
% The final state block uses the Riccati terminal matrix P.

% --- Create the repeated input-change penalty and allocate the state-cost blocks. ---
vehicleCount = params.vehicleCount;
inputChangeWeight = params.inputChangeWeight*eye(vehicleCount);
stateCostBlocks = cell(horizonLength,1);

% --- Build stage-state weights for prediction samples 1 through N-1. ---
for predictionStep = 1:horizonLength-1
    futureHeadway = get_headway(params,currentSample+predictionStep);
    stateCostBlocks{predictionStep} = create_stage_cost(params,futureHeadway);
end

% --- Replace the final stage block with the Riccati terminal cost P. ---
terminalHeadway = get_headway(params,currentSample+horizonLength);
terminalStageCost = create_stage_cost(params,terminalHeadway);
stateCostBlocks{horizonLength} = create_terminal_cost( ...
    model.stateMatrix,model.inputMatrix,terminalStageCost,inputChangeWeight);

% --- Assemble the block-diagonal horizon matrices Omega and Psi. ---
cost.stateWeightMatrix = sparse(blkdiag(stateCostBlocks{:}));
cost.inputChangeWeightMatrix = sparse(kron(eye(horizonLength),inputChangeWeight));
end
