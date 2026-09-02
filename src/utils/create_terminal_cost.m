function terminalCostMatrix = create_terminal_cost(stateMatrix,inputMatrix,stateCostMatrix,inputCostMatrix)
%CREATE_TERMINAL_COST Compute the MPC terminal weight from the discrete Riccati equation.
% Inputs:
%   stateMatrix     : Discrete-time A matrix.
%   inputMatrix     : Discrete-time B matrix.
%   stateCostMatrix : Terminal-stage state weight Q.
%   inputCostMatrix : Input weight R.
% Output:
%   terminalCostMatrix : Riccati solution P used in x'*P*x.
% P is obtained from the discrete algebraic Riccati equation (DARE).

% Solve the DARE and symmetrize P to remove negligible floating-point asymmetry.
terminalCostMatrix = dare(stateMatrix,inputMatrix,stateCostMatrix,inputCostMatrix);
terminalCostMatrix = 0.5*(terminalCostMatrix+terminalCostMatrix.');
end
