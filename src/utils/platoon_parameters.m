function params = platoon_parameters()
%PLATOON_PARAMETERS Define the numerical parameters used by the final simulations.
% Output:
%   params : Structure containing vehicle data, MPC settings, hard constraints,
%            manual-driver events, string-stability disturbance settings, and LQR settings.
% Only parameters used by the cleaned simulation path are retained.

% --- Platoon size, sample time, vehicle geometry, and heterogeneous actuator lags. ---
params.vehicleCount = 5;
params.sampleTime = 0.1;                         % s
params.vehicleLength = 2.5*ones(params.vehicleCount,1); % m
params.actuatorTimeConstant = [0.5;0.2;0.3;0.6;0.4];    % s

% --- Desired speed and initial/final spacing-policy parameters. ---
params.desiredSpeed = 27.78;                     % m/s = 100 km/h
params.standstillDistance = [6;6;5;8;7];        % m
params.initialHeadway = [1.0;0.4;0.2;0.3;0.4]; % s
params.finalHeadway = [1.0;1.9;1.7;1.8;2.0];   % s

% --- Hard gap, speed, and acceleration limits. ---
params.minimumGap = 2;                           % m
params.maximumGap = 70;                          % m
params.minimumSpeed = 0;                         % m/s
params.maximumSpeed = 27.8;                      % m/s
params.minimumAcceleration = -6;                 % m/s^2
params.maximumAcceleration = 3;                  % m/s^2

% --- MPC horizon and cost weights. ---
params.predictionHorizon = 15;
params.referenceRampSamples = 400;
params.relativePositionWeight = 1;
params.absolutePositionWeight = 1;
params.velocityWeight = 1;
params.accelerationWeight = 1;
params.inputChangeWeight = 2;

% --- Manual-driver scenario timing and driver-model gains. ---
params.humanVehicleIndex = 3;
params.emergencyBrakeTime = 100;                 % s
params.stopTime = 150;                           % s
params.lowSpeedTarget = 11;                      % m/s
params.rejoinTime = 250;                         % s
params.humanVelocityGain = 1.2;
params.humanAccelerationGain = 0.8;
params.stopSwitchSpeed = 1.5;                    % m/s

% --- Smooth headway transition from 320 s to 355 s. ---
params.headwayChangeTime = 320;                  % s
params.headwayTransitionDuration = 35;           % s
params.headwayChangeSample = round(params.headwayChangeTime/params.sampleTime);
params.headwayTransitionSamples = round(params.headwayTransitionDuration/params.sampleTime);
params.headwayTransitionEndSample = params.headwayChangeSample + params.headwayTransitionSamples;
params.headwayTransitionEndTime = params.headwayTransitionEndSample*params.sampleTime;

% --- Main-scenario simulation duration. ---
params.mainSimulationFinalTime = 400;             % s

% --- String-stability simulation and disturbance settings. ---
params.stringSimulationFinalTime = 200;           % s
params.stringFigureStartTime = 50;                % s
params.stringAnalysisEndTime = 150;               % s
params.disturbanceVehicleIndex = 1;
params.disturbanceStartTime = 60;                  % s
params.disturbanceEndTime = 120;                   % s
params.disturbancePositionRate = 4.5;              % m/s, reproduction assumption
params.disturbanceEdgeDuration = 3;                % s

% --- LQR comparison settings. ---
params.lqrSimulationFinalTime = 50;                % s
params.lqrRelativePositionWeight = 1;
params.lqrAbsolutePositionWeight = 1;
params.lqrVelocityWeight = 1;
params.lqrInputWeight = 1;
params.lqrDesiredGap = 20.61;                      % m, value aligned with Figure 7
params.lqrLeadInitialPosition = 0;                 % m
end
