% Interpolate each vehicle headway between its initial and final values.
function headway = get_headway(params,sampleIndex)
%GET_HEADWAY Return the headway vector at a specified discrete-time sample.
% Inputs:
%   params      : Project parameters.
%   sampleIndex : Discrete-time sample k.
% Output:
%   headway : Time-headway vector h(k).
% During the scheduled transition, a smoothstep interpolation is used:
%   alpha = 3*s^2 - 2*s^3
%   h = (1-alpha)*h_initial + alpha*h_final

% --- Locate the headway-transition interval in sample coordinates. ---
transitionStart = params.headwayChangeSample;
transitionEnd = params.headwayTransitionEndSample;

% --- Compute a smooth interpolation factor from 0 to 1. ---
if sampleIndex <= transitionStart
    transitionFactor = 0;
elseif sampleIndex >= transitionEnd
    transitionFactor = 1;
else
    normalizedTime = (sampleIndex-transitionStart)/(transitionEnd-transitionStart);
    transitionFactor = 3*normalizedTime^2 - 2*normalizedTime^3;
end

headway = (1-transitionFactor)*params.initialHeadway + transitionFactor*params.finalHeadway;
end
