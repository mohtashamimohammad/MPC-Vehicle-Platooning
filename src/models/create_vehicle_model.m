function [discreteStateMatrix,discreteInputMatrix] = create_vehicle_model(actuatorTimeConstant,sampleTime)
%CREATE_VEHICLE_MODEL Compute the exact ZOH discrete model of one longitudinal vehicle.
% Inputs:
%   actuatorTimeConstant : Actuator lag time constant tau [s].
%   sampleTime           : Sampling period Ts [s].
% Outputs:
%   discreteStateMatrix : A in x(k+1)=A*x(k)+B*u(k).
%   discreteInputMatrix : B in x(k+1)=A*x(k)+B*u(k).
% State:
%   x = [position; velocity; acceleration]
% Continuous dynamics:
%   p_dot = v,  v_dot = a,  a_dot = (u-a)/tau

% --- Compute the exact first-order actuator decay over one sampling period. ---
actuatorDecay = exp(-sampleTime/actuatorTimeConstant);
integratedLag = actuatorTimeConstant*(1-actuatorDecay);

% --- Evaluate the exact zero-order-hold discrete A and B matrices. ---
discreteStateMatrix = [1, sampleTime, actuatorTimeConstant*(sampleTime-integratedLag);
                       0, 1,          integratedLag;
                       0, 0,          actuatorDecay];

discreteInputMatrix = [-actuatorTimeConstant*(sampleTime-integratedLag)+sampleTime^2/2;
                        sampleTime-integratedLag;
                        1-actuatorDecay];
end
