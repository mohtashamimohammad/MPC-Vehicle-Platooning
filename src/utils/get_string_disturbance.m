function disturbance = get_string_disturbance(params,sampleIndex)
%GET_STRING_DISTURBANCE Generate the disturbance used in the string-stability simulation.
% Inputs:
%   params      : Project and disturbance parameters.
%   sampleIndex : Discrete-time sample k.
% Output:
%   disturbance.stateIncrement : Additive state disturbance W(k).
% Simulation model:
%   X(k+1) = A*X(k) + B*U(k) + W(k)
% The disturbance is a smooth position-rate pulse applied to the selected lead vehicle.

% --- Convert the sample index to time and read the disturbance timing parameters. ---
currentTime = sampleIndex*params.sampleTime;
startTime = params.disturbanceStartTime;
endTime = params.disturbanceEndTime;
edgeDuration = params.disturbanceEdgeDuration;

% --- Create a smooth on/off envelope to avoid discontinuous disturbance edges. ---
if currentTime < startTime || currentTime >= endTime
    envelope = 0;
elseif currentTime < startTime+edgeDuration
    normalizedTime = (currentTime-startTime)/edgeDuration;
    envelope = 3*normalizedTime^2-2*normalizedTime^3;
elseif currentTime >= endTime-edgeDuration
    normalizedTime = (endTime-currentTime)/edgeDuration;
    envelope = 3*normalizedTime^2-2*normalizedTime^3;
else
    envelope = 1;
end

% --- Convert the disturbance rate to a per-sample position increment and inject it into one vehicle. ---
positionRate = params.disturbancePositionRate*envelope;
positionIncrement = positionRate*params.sampleTime;
stateIncrement = zeros(3*params.vehicleCount,1);
stateIncrement(params.disturbanceVehicleIndex) = positionIncrement;

disturbance.stateIncrement = stateIncrement;
disturbance.positionRate = positionRate;
end
