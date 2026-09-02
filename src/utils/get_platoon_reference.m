function reference = get_platoon_reference(params,referenceState,sampleIndex)
%GET_PLATOON_REFERENCE Generate the desired platoon state at one sample.
% Inputs:
%   params         : Project parameters.
%   referenceState : Reference-generator state from initialize_reference.
%   sampleIndex    : Discrete-time sample k.
% Output:
%   reference : Desired state, spacing, headway, speed, and acceleration.
% Reference state order:
%   Xref = [p1* ... pM*, v1* ... vM*, a1* ... aM*]'
% The virtual leader accelerates to the desired speed, then continues at constant speed.

% --- Convert the requested sample to elapsed time from the latest reference initialization. ---
vehicleCount = params.vehicleCount;
elapsedSamples = sampleIndex-referenceState.startSample;
elapsedTime = elapsedSamples*params.sampleTime;
rampDuration = referenceState.rampDuration;
rampAcceleration = referenceState.rampAcceleration;

% --- Generate the virtual-leader speed, acceleration, and position. ---
if elapsedSamples < params.referenceRampSamples
    referenceAcceleration = rampAcceleration;
    referenceSpeed = referenceState.initialSpeed + rampAcceleration*elapsedTime;
    virtualLeaderPosition = referenceState.virtualLeaderPosition + ...
        referenceState.initialSpeed*elapsedTime + 0.5*rampAcceleration*elapsedTime^2;
else
    referenceAcceleration = 0;
    referenceSpeed = params.desiredSpeed;
    positionAtRampEnd = referenceState.virtualLeaderPosition + ...
        referenceState.initialSpeed*rampDuration + 0.5*rampAcceleration*rampDuration^2;
    virtualLeaderPosition = positionAtRampEnd + params.desiredSpeed*(elapsedTime-rampDuration);
end

% --- Convert the virtual-leader trajectory into desired positions for all platoon vehicles. ---
headway = get_headway(params,sampleIndex);
desiredSpacing = calculate_desired_spacing(params,referenceSpeed,headway);
referencePosition = virtualLeaderPosition-cumsum(desiredSpacing);
referenceStateVector = [referencePosition; ...
    referenceSpeed*ones(vehicleCount,1); ...
    referenceAcceleration*ones(vehicleCount,1)];

% --- Package the desired state and spacing information for the MPC. ---
reference.stateVector = referenceStateVector;
reference.position = referencePosition;
reference.speed = referenceSpeed;
reference.acceleration = referenceAcceleration;
reference.desiredSpacing = desiredSpacing;
reference.headway = headway;
end
