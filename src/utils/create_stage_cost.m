% --- Assemble Q_k in [position; velocity; acceleration] block form. ---
function stageCostMatrix = create_stage_cost(params,headway)
%CREATE_STAGE_COST Build the single-sample state-error weighting matrix Q_k.
% Inputs:
%   params  : Project parameters and MPC weighting coefficients.
%   headway : Headway vector used at the evaluated sample.
% Output:
%   stageCostMatrix : Q_k for state error [position; velocity; acceleration].
% Q_k combines relative-position, absolute-position, velocity, and acceleration penalties.
% The headway terms couple position and velocity errors through the spacing policy.

% --- Build reusable identity and zero blocks. ---
vehicleCount = params.vehicleCount;
identityVehicles = eye(vehicleCount);
zeroVehicles = zeros(vehicleCount);

% --- Construct the neighboring-vehicle position-error penalty matrix. ---
relativePositionMatrix = 2*identityVehicles + ...
    diag(-ones(vehicleCount-1,1),1) + diag(-ones(vehicleCount-1,1),-1);

% --- Build headway-dependent position/velocity coupling terms. ---
headwayCouplingMatrix = diag(headway) + diag(-headway(2:end),1);
headwaySquaredMatrix = diag(headway.^2);

stageCostMatrix = [params.relativePositionWeight*relativePositionMatrix + ...
                   params.absolutePositionWeight*identityVehicles, ...
                   params.relativePositionWeight*headwayCouplingMatrix, ...
                   zeroVehicles;
                   params.relativePositionWeight*headwayCouplingMatrix.', ...
                   params.relativePositionWeight*headwaySquaredMatrix + ...
                   params.velocityWeight*identityVehicles, ...
                   zeroVehicles;
                   zeroVehicles, zeroVehicles, ...
                   params.accelerationWeight*identityVehicles];

stageCostMatrix = 0.5*(stageCostMatrix+stageCostMatrix.');
end
