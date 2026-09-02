function solution = solve_mpc_qp(qp)
%SOLVE_MPC_QP Solve a constrained convex quadratic program and reconstruct Xpred.
% Input:
%   qp.H, qp.f : Objective matrices.
%   qp.A, qp.b : Linear inequalities A*z <= b.
%   qp.basePredictedState   : Xbase in Xpred = Xbase + Bdecision*z.
%   qp.decisionEffectMatrix : Bdecision in the predicted-state equation.
% Output:
%   solution.decision       : Optimal decision vector z.
%   solution.predictedState : Predicted state stack produced by z.
% QP form:
%   min 0.5*z'*H*z + f'*z
%   subject to A*z <= b
% quadprog display output is disabled to keep the MATLAB Command Window clean.

% Solve for the optimal decision vector z.
% --- Configure quadprog for a silent solve without changing its default numerical settings. ---
solverOptions = optimoptions('quadprog','Display','off');
optimalDecision = quadprog(qp.H,qp.f,qp.A,qp.b,[],[],[],[],[],solverOptions);

% Reconstruct the full predicted state trajectory from the optimal decision.
predictedState = qp.basePredictedState + qp.decisionEffectMatrix*optimalDecision;

% --- Return the optimizer decision and predicted state. ---
solution.decision = optimalDecision;
solution.predictedState = predictedState;
end
