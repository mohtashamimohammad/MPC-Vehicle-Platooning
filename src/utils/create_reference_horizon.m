function referenceHorizon = create_reference_horizon(params,referenceState,currentSample,horizonLength)
%CREATE_REFERENCE_HORIZON Stack the desired platoon states over the next N samples.
% Inputs:
%   params         : Project parameters.
%   referenceState : Reference-generator state from initialize_reference.
%   currentSample  : Current sample k.
%   horizonLength  : Prediction horizon N.
% Output:
%   referenceHorizon.stateStack : [X*(k+1); ...; X*(k+N)].

% --- Allocate the stacked reference vector for N future states. ---
stateCount = 3*params.vehicleCount;
referenceStateStack = zeros(horizonLength*stateCount,1);

% --- Generate X*(k+j) for each prediction sample and place it in the stacked vector. ---
for predictionStep = 1:horizonLength
    futureReference = get_platoon_reference( ...
        params,referenceState,currentSample+predictionStep);
    stateRows = (predictionStep-1)*stateCount + (1:stateCount);
    referenceStateStack(stateRows) = futureReference.stateVector;
end

referenceHorizon.stateStack = referenceStateStack;
end
