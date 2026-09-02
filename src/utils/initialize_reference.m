function referenceState = initialize_reference(params,currentState,startSample,anchorVehicleIndex)
%INITIALIZE_REFERENCE Initialize the virtual-leader reference generator.
% Inputs:
%   params             : Project parameters.
%   currentState       : Current full platoon state X=[p;v;a].
%   startSample        : Sample k0 at which the reference is initialized.
%   anchorVehicleIndex : 0 for normal operation; vehicle index when the reference is anchored to a manual vehicle.
% Output:
%   referenceState : Data required by get_platoon_reference.
% Normal operation starts from the minimum platoon speed.
% During manual mode, the reference speed and position are anchored to the human-driven vehicle.

% Use normal virtual-leader anchoring when no explicit anchor vehicle is supplied.
if nargin < 4
    anchorVehicleIndex = 0;
end

% --- Extract current positions and velocities from the centralized state vector. ---
vehicleCount = params.vehicleCount;
positions = currentState(1:vehicleCount);
velocities = currentState(vehicleCount+1:2*vehicleCount);

% --- Select the initial reference speed from either the platoon or the manual vehicle. ---
if anchorVehicleIndex == 0
    initialReferenceSpeed = min(velocities);
else
    initialReferenceSpeed = velocities(anchorVehicleIndex);
end

% --- Compute the desired spacing at reference initialization. ---
initialHeadway = get_headway(params,startSample);
initialSpacing = calculate_desired_spacing(params,initialReferenceSpeed,initialHeadway);

% --- Place the virtual leader so the selected anchor is consistent with the desired spacing policy. ---
if anchorVehicleIndex == 0
    virtualLeaderPosition = positions(1) + initialSpacing(1);
else
    virtualLeaderPosition = positions(anchorVehicleIndex) + ...
        sum(initialSpacing(1:anchorVehicleIndex));
end

% --- Store the reference ramp parameters used by get_platoon_reference. ---
referenceState.startSample = startSample;
referenceState.initialSpeed = initialReferenceSpeed;
referenceState.virtualLeaderPosition = virtualLeaderPosition;
referenceState.rampAcceleration = ...
    (params.desiredSpeed-initialReferenceSpeed)/(params.sampleTime*params.referenceRampSamples);
referenceState.rampDuration = params.sampleTime*params.referenceRampSamples;
end
