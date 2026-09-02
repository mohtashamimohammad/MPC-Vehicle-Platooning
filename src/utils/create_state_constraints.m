function constraints = create_state_constraints(params,horizonLength)
%CREATE_STATE_CONSTRAINTS Build the hard gap, speed, and acceleration inequalities.
% Inputs:
%   params        : Project limits.
%   horizonLength : Prediction horizon N.
% Output:
%   constraints : Per-step and lifted inequalities in the form G*X + g <= 0.
% Physical constraints:
%   dMin <= p_i-p_(i+1) <= dMax
%   vMin <= v_i <= vMax
%   aMin <= a_i <= aMax

% --- Build basic matrix blocks for one prediction sample. ---
vehicleCount = params.vehicleCount;
identityVehicles = eye(vehicleCount);
zeroVehicles = zeros(vehicleCount);
zeroGapBlock = zeros(vehicleCount-1,vehicleCount);

% --- Create the neighbor-position difference operator. ---
gapDifferenceMatrix = zeros(vehicleCount-1,vehicleCount);
for gapIndex = 1:vehicleCount-1
    gapDifferenceMatrix(gapIndex,gapIndex) = -1;
    gapDifferenceMatrix(gapIndex,gapIndex+1) = 1;
end

% --- Encode gap, speed, and acceleration limits as G_step*X + g_step <= 0. ---
perStepMatrix = [ gapDifferenceMatrix,  zeroGapBlock,       zeroGapBlock;
                 -gapDifferenceMatrix,  zeroGapBlock,       zeroGapBlock;
                  zeroVehicles,        -identityVehicles,   zeroVehicles;
                  zeroVehicles,         identityVehicles,   zeroVehicles;
                  zeroVehicles,         zeroVehicles,      -identityVehicles;
                  zeroVehicles,         zeroVehicles,       identityVehicles];

perStepOffset = [ params.minimumGap*ones(vehicleCount-1,1);
                 -params.maximumGap*ones(vehicleCount-1,1);
                  params.minimumSpeed*ones(vehicleCount,1);
                 -params.maximumSpeed*ones(vehicleCount,1);
                  params.minimumAcceleration*ones(vehicleCount,1);
                 -params.maximumAcceleration*ones(vehicleCount,1)];

% --- Repeat the one-step inequalities over all N prediction samples. ---
constraints.perStepMatrix = sparse(perStepMatrix);
constraints.perStepOffset = perStepOffset;
constraints.horizonMatrix = kron(speye(horizonLength),sparse(perStepMatrix));
constraints.horizonOffset = repmat(perStepOffset,horizonLength,1);
end
