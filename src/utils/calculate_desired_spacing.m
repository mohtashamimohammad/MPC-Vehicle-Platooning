% Evaluate the desired spacing for all vehicles at the supplied reference speed.
function desiredSpacing = calculate_desired_spacing(params,referenceSpeed,headway)
%CALCULATE_DESIRED_SPACING Compute the desired longitudinal spacing for every vehicle.
% Inputs:
%   params         : Project parameters from platoon_parameters.
%   referenceSpeed : Current reference speed v_ref [m/s].
%   headway        : Time-headway vector h_i [s], one value per vehicle.
% Output:
%   desiredSpacing : Desired predecessor spacing d_i [m].
% Equation:
%   d_i = L_(i-1) + r_i + h_i*v_ref
% The first entry is the spacing from vehicle 1 to the virtual leader.

% Build the predecessor-length vector used by the constant-time-headway spacing rule.
predecessorLength = [params.vehicleLength(1); params.vehicleLength(1:end-1)];
desiredSpacing = predecessorLength + params.standstillDistance + headway*referenceSpeed;
end
