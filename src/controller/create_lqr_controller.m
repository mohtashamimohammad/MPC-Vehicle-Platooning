function controller = create_lqr_controller(params)
%CREATE_LQR_CONTROLLER Build the continuous-time LQR controller used for comparison.
% Input:
%   params : Project parameters and LQR weighting coefficients.
% Output:
%   controller.gain : LQR gain K used in u = -K*(x-x_ref).
% Comparison model:
%   p_dot = v,  v_dot = u
% State order:
%   x = [p1 ... pM, v1 ... vM]'
% LQR cost:
%   integral(x'*Q*x + u'*R*u) dt

% --- Build the continuous-time double-integrator platoon model. ---
vehicleCount = params.vehicleCount;
identityVehicles = eye(vehicleCount);
zeroVehicles = zeros(vehicleCount);

stateMatrix = [zeroVehicles,identityVehicles;
               zeroVehicles,zeroVehicles];
inputMatrix = [zeroVehicles;identityVehicles];

% --- Penalize relative position error between neighboring vehicles. ---
relativePositionMatrix = 2*identityVehicles + ...
    diag(-ones(vehicleCount-1,1),1) + diag(-ones(vehicleCount-1,1),-1);

% --- Assemble the LQR Q and R matrices and compute the optimal feedback gain. ---
stateCostMatrix = blkdiag( ...
    params.lqrRelativePositionWeight*relativePositionMatrix + ...
    params.lqrAbsolutePositionWeight*identityVehicles, ...
    params.lqrVelocityWeight*identityVehicles);
inputCostMatrix = params.lqrInputWeight*identityVehicles;

controller.gain = lqr(stateMatrix,inputMatrix,stateCostMatrix,inputCostMatrix);
end
